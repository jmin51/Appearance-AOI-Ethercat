#ifndef AXIS_PROBE_HPP
#define AXIS_PROBE_HPP

// ============================================================================
// 轴指令探针（Axis Probe）
//
// 目的：抓"指令位置(0x607A)单周期异常跳变"——即现场那种
//      "一直 -655 脉冲/周期朝负方向走，突然某一个周期变成 +1310"
//      的现象。
//
// 原理：每个 EtherCAT 周期（1ms）在 handle_state_machine() 的**末尾**回读
//      本轴实际写进 0x607A 的指令位置，与上一周期比较：
//        |Δcmd| > 本周期物理允许步进  →  判定为异常跳变，打印现场 + 回放。
//      这样【无论哪条代码路径】（自动插补 / 状态机强制对齐 / 限位 / 停止 /
//      回零 / 初始化）改写了指令位置，都会被同一处捕获，不会漏。
//
// 线程模型：只由 EtherCAT 实时线程访问，无锁、无动态分配；
//          正常周期只做几次整数比较（≈ 纳秒级），异常才 printf。
// ============================================================================

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>

namespace axis_probe
{

// 指令位置本周期是由哪条路径写入的（用于归类异常来源）
enum class Src : uint8_t
{
    UNKNOWN       = 0,
    GRADUAL       = 1,   // 自动模式梯形插补 gradual_approach（正常路径）
    MANUAL_JOG    = 2,   // 手动点动
    ALIGN_CURRENT = 3,   // 状态机把指令位置直接对齐到实际编码器位置
    HOLD          = 4,   // READY / STOPPED 保持当前位置
    HOMING        = 5,   // 回零
    INIT          = 6,   // 初始化 / 故障清除
};

inline const char* src_name(uint8_t s)
{
    switch (static_cast<Src>(s))
    {
        case Src::GRADUAL:       return "GRADUAL(梯形插补)";
        case Src::MANUAL_JOG:    return "MANUAL_JOG(点动)";
        case Src::ALIGN_CURRENT: return "ALIGN_CURRENT(对齐实际位置)";
        case Src::HOLD:          return "HOLD(保持)";
        case Src::HOMING:        return "HOMING(回零)";
        case Src::INIT:          return "INIT(初始化)";
        default:                 return "UNKNOWN";
    }
}

// 统一用 CLOCK_MONOTONIC，便于与伺服/示波器/其它日志按"运行时长"对齐
inline uint64_t now_ns()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL
         + static_cast<uint64_t>(ts.tv_nsec);
}

// 单调时钟 → "秒.毫秒"（打印用，不做 I/O）
inline void fmt_ts(uint64_t ts_ns, char* buf, size_t len)
{
    std::snprintf(buf, len, "%llu.%03llu",
                  static_cast<unsigned long long>(ts_ns / 1000000000ULL),
                  static_cast<unsigned long long>((ts_ns / 1000000ULL) % 1000ULL));
}

// 一个周期的指令现场快照
struct Snapshot
{
    uint64_t ts_ns      = 0;
    int32_t  cmd        = 0;   // 本周期写入 0x607A 的指令位置（脉冲）
    int32_t  target     = 0;   // target_pulses_（本次运动终点）
    int32_t  rem        = 0;   // target - cmd
    int32_t  cur_pos    = 0;   // 实际编码器 0x6064（脉冲）
    int32_t  allow      = 0;   // 本周期物理允许步进（脉冲）
    uint16_t status_word = 0;  // 0x6041
    uint16_t error_code  = 0;  // 0x603F
    double   prof_v     = 0.0; // 梯形剖面当前速度（脉冲/周期）
    double   prof_frac  = 0.0; // 位置余数
    double   tgt_disp   = 0.0; // 上位机下发的工程值（mm / °）
    uint8_t  src        = 0;
    uint8_t  state      = 0;   // AxisState
    uint8_t  flags      = 0;   // bit0 displacement_updated_ / bit1 position_initialized_
                               // bit2 stop_requested_ / bit3 prof_reset_requested_
};

class Probe
{
public:
    // 环形回放深度：异常时把最近 kRing 个周期打出来（含异常前 N 个周期）
    // 用 enum 而非 static constexpr，避免 C++11/14 下 ODR-use 需要类外定义
    enum : int { kRing = 24 };

    void set_name(const char* n)
    {
        if (n && *n) name_ = n;
    }

    // 只入环形缓冲、不参与比较（例如插补函数内部想留一帧快照时使用）
    void sample(const Snapshot& sn)
    {
        ring_[ring_idx_] = sn;
        ring_idx_ = (ring_idx_ + 1) % kRing;
        if (total_ < static_cast<uint32_t>(kRing)) ++total_;
    }

    // 主入口：入环形缓冲 + 与上一周期比较，超限即报现场
    void record(const Snapshot& sn, int32_t allow_step)
    {
        const bool have_prev = has_prev_;
        const Snapshot prev  = prev_;

        sample(sn);

        if (have_prev && allow_step > 0)
        {
            const int64_t delta = static_cast<int64_t>(sn.cmd)
                                - static_cast<int64_t>(prev.cmd);
            if (std::llabs(delta) > static_cast<int64_t>(allow_step))
            {
                report(prev, sn, delta, allow_step);
            }
        }

        prev_     = sn;
        has_prev_ = true;
    }

    uint32_t jump_count() const { return jump_cnt_; }

private:
    void report(const Snapshot& prev, const Snapshot& cur,
                int64_t delta, int32_t allow_step)
    {
        ++jump_cnt_;

        char t_prev[32];
        char t_cur[32];
        fmt_ts(prev.ts_ns, t_prev, sizeof(t_prev));
        fmt_ts(cur.ts_ns,  t_cur,  sizeof(t_cur));

        // 打印速度受 ECM 周期限制：异常本身很稀疏，不做节流，
        // 但同一微秒内重复触发时只留第一条，避免刷爆终端
        if (cur.ts_ns - last_report_ns_ < 200000ULL) return;
        last_report_ns_ = cur.ts_ns;

        std::printf(
            "\n"
            "==================== [指令跳变] 轴 %s ====================\n"
            "  Δcmd = %+lld 脉冲（本周期物理上限 %d）\n"
            "  本周期  t=%s cmd=%d target=%d rem=%d 实际=%d v=%.1f 状态字=0x%04X 错误码=0x%04X\n"
            "          state=%u src=%s flags=0x%02X(upd=%d,init=%d,stop=%d,rst=%d) 下发值=%.6f\n"
            "  上一周期 t=%s cmd=%d target=%d rem=%d 实际=%d v=%.1f\n"
            "  --- 最近 %u 个周期回放（Δ=与上一周期差值） ---\n",
            name_,
            static_cast<long long>(delta), allow_step,
            t_cur, cur.cmd, cur.target, cur.rem, cur.cur_pos, cur.prof_v,
            cur.status_word, cur.error_code,
            static_cast<unsigned>(cur.state), src_name(cur.src), cur.flags,
            (cur.flags & 0x01) ? 1 : 0, (cur.flags & 0x02) ? 1 : 0,
            (cur.flags & 0x04) ? 1 : 0, (cur.flags & 0x08) ? 1 : 0,
            cur.tgt_disp,
            t_prev, prev.cmd, prev.target, prev.rem, prev.cur_pos, prev.prof_v,
            total_);

        // 按时间顺序回放环形缓冲
        const uint32_t n  = total_;
        const uint32_t st = (total_ >= static_cast<uint32_t>(kRing))
                          ? ring_idx_ : 0;
        for (uint32_t k = 0; k < n; ++k)
        {
            const Snapshot& s = ring_[(st + k) % kRing];
            char ts[32];
            fmt_ts(s.ts_ns, ts, sizeof(ts));
            std::printf("    t=%s cmd=%-8d target=%-8d 实际=%-8d v=%8.1f st=%u src=%s\n",
                        ts, s.cmd, s.target, s.cur_pos, s.prof_v,
                        static_cast<unsigned>(s.state), src_name(s.src));
        }
        std::printf("==========================================================\n\n");
        std::fflush(stdout);
    }

    const char* name_ = "?";
    Snapshot    ring_[kRing];
    uint32_t    ring_idx_ = 0;
    uint32_t    total_    = 0;
    Snapshot    prev_{};
    bool        has_prev_ = false;
    uint32_t    jump_cnt_ = 0;
    uint64_t    last_report_ns_ = 0;
};

} // namespace axis_probe

#endif // AXIS_PROBE_HPP
