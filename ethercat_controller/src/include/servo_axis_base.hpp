#ifndef SERVO_AXIS_BASE_HPP
#define SERVO_AXIS_BASE_HPP

#include <ecrt.h>
#include <string>
#include <memory>
#include <mutex>
#include <atomic>
#include <rclcpp/rclcpp.hpp>

// 状态机状态定义
enum class AxisState {
    UNINITIALIZED = 0,
    INITIALIZING = 1,
    READY = 2,
    MANUAL_MODE = 3,
    AUTO_MODE = 4,
    STOPPED = 5,
    FAULT = 6
};

enum class OperationMode {
    MANUAL = 0,
    AUTO = 1
};

enum class AxisType {
    AXIS1 = 0,
    AXIS2 = 1
};

enum class DriveBrand {
    LEISAI = 0,
    HUICHUAN = 1
};

// 轴计算模式：决定该轴使用位移模式还是角度模式进行计算
enum class AxisCalcMode {
    DISPLACEMENT = 0,  // 位移模式 (mm)
    ANGLE = 1          // 角度模式 (rad)
};

// CiA402 PDS 状态：由状态字 0x6041 的低 8 位解码得到
// 解码只使用 bit0~3、bit5、bit6，bit7(warning) 与 bit8~15(厂商位) 全部忽略，
// 因此 0x0250/0x0650/0x1650/0x2650/0x0670/0x0770 等都会归为同一个 SWITCH_ON_DISABLED
enum class PdsState {
    NOT_READY = 0,          // Not ready to switch on
    SWITCH_ON_DISABLED,     // Switch on disabled（未使能）
    READY_TO_SWITCH_ON,     // Ready to switch on
    SWITCHED_ON,            // Switched on
    OPERATION_ENABLED,      // Operation enabled（已使能）
    QUICK_STOP_ACTIVE,      // Quick stop active
    FAULT_REACTION_ACTIVE,  // Fault reaction active
    FAULT,                  // Fault
    UNKNOWN                 // 未定义/过渡状态（如 0x0e00/0x0101 等上电瞬态）
};

class ServoAxisBase {
public:
    ServoAxisBase(const std::string& name, uint16_t position, AxisType axis_type, DriveBrand brand, uint32_t product_code, double gear_ratio = 1.0, AxisCalcMode calc_mode = AxisCalcMode::DISPLACEMENT);
    virtual ~ServoAxisBase() = default;
    // 添加获取产品号的虚函数
    virtual uint32_t get_product_code() const = 0;

    // 纯虚函数 - 必须由子类实现
    virtual void configure(ec_master_t* master) = 0;
    virtual void register_pdo_entries(ec_pdo_entry_reg_t* reg_list, int& index) = 0;
    virtual void handle_state_machine(uint8_t* domain1_pd) = 0;
    
    // 虚函数 - 可以有默认实现
    virtual void set_target_displacement(double displacement);
    virtual void set_displacement_updated(bool updated);
    virtual void stop();
    virtual void start_manual_mode();
    virtual void start_auto_mode();
    virtual void clear_fault();
    virtual void reset_axis();
    virtual void reset_motion_state();  // 重置运动状态

    // 获取函数
    virtual std::string get_name() const;

    // 获取错误代码（必须实现）
    virtual uint16_t get_error_code() const = 0;
    virtual AxisState get_current_state() const;
    
    virtual OperationMode get_operation_mode() const;
    virtual bool is_ready() const;
    virtual bool is_running() const;
    virtual bool is_homing_completed() const;
    virtual bool is_homing_in_progress() const;
    virtual ec_slave_config_t* get_slave_config();
    virtual unsigned int get_control_word_offset() const;
    virtual int32_t get_actual_position() const;
    virtual int32_t get_initial_position() const;
    virtual int32_t get_raw_encoder_pulses() const { return raw_encoder_pulses_.load(); }
    virtual DriveBrand get_brand() const;
    virtual AxisCalcMode get_calc_mode() const { return calc_mode_; }

    // CiA402 状态字解码：屏蔽厂商位，统一各型号/固件的状态跳转判断
    // 使用标准掩码 0x4F(bits0~3,6) / 0x6F(bits0~3,5,6)
    static PdsState decode_pds_state(uint16_t status_word);
    static const char* pds_state_name(PdsState state);

    // 回原轴标识（由 YAML operation_mode 字段决定，替代硬编码轴名判断）
    virtual void set_homing_mode(bool on) { homing_mode_ = on; }
    virtual bool is_homing_mode() const { return homing_mode_; }

    // ========== 回原达标（仅 HM 回原会话有效） ==========
    // 背景：复位/回原流程中回原轴按 0x6060=6(HM) 配置，驱动器会自行执行回原，
    //       完成瞬间把状态字置为"使能 + bit12(Homing attained)"（汇川实测为 0x1637）。
    //       若只看 PDS 阶段，轴会在 0x0633(Switched on) 就被判 READY → 上层"系统就绪"→
    //       "回原完成→待启动"立刻 safe_shutdown，把正在进行的回原打断（必须靠 bit12 拦截）。
    // 会话边界：由子类 configure() 调用 begin_homing_session() 设置/清除，天然按每次重启划分。
    virtual bool is_homing_required() const { return homing_required_.load(std::memory_order_acquire); }
    virtual bool is_homing_attained() const { return homing_attained_.load(std::memory_order_acquire); }
    // 开始/结束一次回原会话（required=true 表示本次为 HM 会话，必须等回原达标）
    virtual void begin_homing_session(bool required);
    // 实时线程每周期调用：判定"使能 + bit12 连续稳定"，达标后锁存（诊断/上层判定共用）
    virtual void update_homing_attainment(PdsState pds, uint16_t raw_status_word);
    
    // 每转脉冲数（编码器分辨率）设置/获取
    virtual void set_pulses_per_rev(int32_t ppr) { pulses_per_rev_ = ppr; }
    virtual int32_t get_pulses_per_rev() const { return pulses_per_rev_; }
    // 添加点动控制方法
    virtual void jog_forward() { 
        jog_forward_requested_ = true;
        jog_reverse_requested_ = false;
        jog_stop_requested_ = false;
    }
    
    virtual void jog_reverse() {
        jog_reverse_requested_ = true; 
        jog_forward_requested_ = false;
        jog_stop_requested_ = false;
    }
    
    virtual void jog_stop() {
        jog_stop_requested_ = true;
        jog_forward_requested_ = false;
        jog_reverse_requested_ = false;
    }
    
    virtual bool is_jogging() const {
        return jog_forward_requested_ || jog_reverse_requested_;
    }
    // 脉冲 <-> 工程单位换算（公开，供外部如 joint_states 反馈使用）
    virtual int32_t displacement_to_pulses(double displacement_mm);
    virtual double pulses_to_displacement(int32_t pulses);
    virtual int32_t angle_to_pulses(double angle_degree);
    virtual double pulses_to_angle(int32_t pulses);
    bool check_target_reached_flag();   // 检查目标到达标志（一次性读取，读取后自动清除）
    bool is_target_reached() const;     // 检查是否到达目标位置（持续状态）
    // 添加点动速度设置方法
    virtual bool set_jog_speed(double speed);
    virtual double get_jog_speed() const { return jog_speed_; }
    // 速度范围设置方法（由YAML配置驱动）
    virtual void set_min_jog_speed(double min_speed) { min_jog_speed_ = min_speed; }
    virtual void set_max_jog_speed(double max_speed) { max_jog_speed_ = max_speed; }
    virtual double get_min_jog_speed() const { return min_jog_speed_; }
    virtual double get_max_jog_speed() const { return max_jog_speed_; }

    // 自动模式速度设置方法（由YAML配置驱动）
    // 校验规则与 set_move_speed 一致: 0 < speed <= max_jog_speed_ 且每周期步长 >= 1
    // 依赖 max_jog_speed_ 已完成配置（load_axes_from_yaml 中先设 max_jog_speed 再设 auto_speed）
    virtual bool set_auto_speed(double speed);
    virtual double get_auto_speed() const { return auto_speed_; }

    // 本次移动速度（一次性）：设置后仅对紧随其后的一次移动生效，未设置时回落到 auto_speed_
    // 供 /displacement_command_pct、/angle_command_pct 使用；设置失败返回 false，调用方应拒绝该轴指令
    virtual bool set_move_speed(double speed);
    virtual void clear_move_speed();
    virtual bool has_move_speed() const { return has_move_speed_.load(std::memory_order_acquire); }
    virtual double get_move_speed() const { return move_speed_.load(std::memory_order_relaxed); }

    // 自动模式梯形速度曲线加减速参数（由YAML配置驱动）
    // 单位与 auto_speed 一致（位移轴 mm/s²、角度轴 °/s²）；decel<=0 时取 accel
    // 建议默认取 4 × max_jog_speed，使任何速度百分比下都能在 0.25s 内达到巡航速度
    // 返回 false 表示 accel<=0 非法，调用方应忽略并告警
    virtual bool set_auto_profile(double accel, double decel);
    virtual double get_accel() const { return accel_; }
    virtual double get_decel() const { return decel_; }

    // 手动点动速度曲线加减速参数（由YAML配置驱动，单位与 accel/decel 一致）
    // 缺省（不配置时由 manual_jog_profile 现算）：
    //   jog_accel = 4 × jog_speed      → 0 → 点动速度约 0.25s（软起动）
    //   jog_decel = 20 × max_jog_speed → 全速刹停 2.5~15ms，滑行小（软停止）
    // 传 0 表示该项沿用缺省；负值非法被忽略并告警，返回 false
    virtual bool set_jog_profile(double accel, double decel);
    virtual double get_jog_accel() const { return jog_accel_; }
    virtual double get_jog_decel() const { return jog_decel_; }

    // 角度软限位方法（仅角度模式轴有效，单位：度）
    virtual void set_angle_limits(double min_deg, double max_deg);
    virtual double get_min_angle() const { return min_angle_deg_; }
    virtual double get_max_angle() const { return max_angle_deg_; }
    virtual bool has_angle_limits() const { return has_angle_limits_; }
    // 将目标角度钳制到软限位范围内（未启用限位则原值返回）
    virtual double clamp_angle(double angle_deg) const;

protected:
    // 保护成员变量 - 子类可以访问
    std::string axis_name_;
    uint16_t slave_position_;
    AxisType axis_type_;
    DriveBrand brand_;
    uint32_t product_code_;  // 在基类中存储产品号
    double gear_ratio_; // 添加减速比成员变量
    AxisCalcMode calc_mode_; // 轴计算模式
    bool homing_mode_ = false; // 是否为回原轴（YAML operation_mode="homing"）
    // 回原达标状态（HM 会话）：实时线程写、主线程读（灯语/就绪判定）
    std::atomic<bool> homing_required_{false};   // 本会话按 HM 配置，必须等回原完成
    std::atomic<bool> homing_attained_{false};   // 本会话已见 bit12（锁存，会话开始/重置时清除）
    int homing_attained_stable_count_ = 0;       // bit12 连续成立的周期数（仅实时线程访问）
    // bit12 需连续成立的周期数：约20ms @1kHz，滤掉驱动器状态字抖动（实测会在 0x1637/0x1638/0x0 间跳）
    static constexpr int kHomingAttainedStableCycles = 20;
    
    ec_slave_config_t* sc_;
    unsigned int control_word_;
    unsigned int status_word_;
    unsigned int off_target_position_;
    unsigned int off_actual_position_;
    unsigned int off_error_code_;
    
    AxisState current_state_;
    OperationMode operation_mode_;
    
    bool position_initialized_;
    volatile int32_t target_pulses_;  // 告诉编译器"这个变量可能被外部访问"
    double target_displacement_;
    bool displacement_updated_;
    int32_t joint_position_;
    int32_t initial_position_;
    std::atomic<int32_t> raw_encoder_pulses_{0};  // 绝对编码器原始脉冲（线程安全，供/joint_states反馈）
    int32_t target_reached_;
    
    bool homing_in_progress_;
    bool homing_completed_;
    
    // 控制标志
    bool start_manual_requested_;
    bool start_auto_requested_;
    bool stop_requested_;        // 停止请求（结束作业时先减速再跳转）
    bool clear_fault_requested_;
    bool reset_requested_;
    bool fault_clearing_in_progress_;
    int fault_clear_step_;
    int fault_clear_counter_;

    // 保护方法 - 子类可以重写或使用
    virtual void initialize_members();
    virtual void check_state_changes(uint16_t read_status_word, uint16_t error_code);
    virtual void handle_fault_clear(uint8_t* domain1_pd);
    virtual void check_system_initialization();
    // 速度合法性统一校验（set_auto_speed / set_move_speed 共用）
    // 规则: 0 < speed <= max_jog_speed_ 且折算到每周期脉冲 >= 1
    // tag: 日志前缀，如 "自动速度" / "本次移动速度"
    virtual bool validate_speed(double speed, const char* tag);
    // 清残留的运动指令标志（切换模式时调用）
    // displacement_updated_ / jog_*_requested_ 都是"只置不清"：
    //   自动模式下点动标志不被消费（或手动模式下位移标志不被消费），
    //   若不在换模式时清掉，新模式的第一个周期会照残留指令突然运动。
    virtual void clear_pending_motion_requests();
    // 工程单位 → 脉冲 的换算系数（double，不做整数截断）
    // 位移轴: 脉冲/mm ; 角度轴: 脉冲/°
    // 注意: 不能复用 displacement_to_pulses()/angle_to_pulses()，它们返回 int32_t，
    //       会把加速度(如 0.432 脉冲/周期²)截断成 0，导致梯形曲线完全失效
    virtual double pulses_per_unit() const;
    // 添加获取减速比的方法
    virtual double get_gear_ratio() const { return gear_ratio_; }
    virtual void set_gear_ratio(double ratio) { gear_ratio_ = ratio; }

    // 每转脉冲数（编码器分辨率），默认36000
    int32_t pulses_per_rev_ = 36000;

    // 速度控制相关
    double jog_speed_;           // 点动速度 (mm/s)
    bool jog_forward_requested_; // 正转请求
    bool jog_reverse_requested_; // 反转请求
    bool jog_stop_requested_;    // 停止请求
    
    // 限位触发方向记录（Bit11响应用）
    // 0: 未触发  1: 正限位方向  -1: 负限位方向
    int limit_trigger_direction_ = 0;
    
    // 添加速度设置互斥锁
    mutable std::mutex speed_mutex_;

    // 速度限制参数（由YAML配置设定，不同轴可有不同范围）
    // 位移轴: 单位 mm/s; 角度轴: 单位 °/s
    double min_jog_speed_ = 1.0;      // 最小点动速度
    double max_jog_speed_ = 200.0;    // 最大点动速度
    const double DEFAULT_JOG_SPEED = 1.0; // 默认点动速度

    // 自动模式速度参数（由YAML配置设定）
    // 位移轴: 单位 mm/s; 角度轴: 单位 °/s
    double auto_speed_;                // 自动模式移动速度（默认速度）
    const double DEFAULT_AUTO_SPEED = 5.0; // 默认自动速度

    // 自动模式梯形速度曲线参数（位移轴 mm/s²、角度轴 °/s²）
    // 0 表示未配置 → gradual_approach 退回"限速一步式"（无加减速，保证轴仍能运动）
    double accel_ = 0.0;               // 加速度上限
    double decel_ = 0.0;               // 减速度上限（<=0 时取 accel_）

    // 本次移动速度（供 /xxx_command_pct 使用）
    // 写方=ROS回调线程，读方=EtherCAT实时线程(1kHz)，故用原子量，勿用裸 double；
    // 先写 move_speed_ 再置 has_move_speed_(release)，读方以 acquire 读标志
    std::atomic<double> move_speed_{0.0};
    std::atomic<bool>   has_move_speed_{false};

    // 关节角度软限位（仅角度模式轴有效，单位：度）
    // 当收到角度指令时，目标角度将被钳制在 [min_angle_deg_, max_angle_deg_] 范围内
    double min_angle_deg_ = -180.0;
    double max_angle_deg_ = 180.0;
    bool has_angle_limits_ = false;    // 是否启用软限位
    
    // 在保护成员变量中添加
    std::atomic<bool> target_reached_flag_{false};
    mutable std::mutex flag_mutex_;

    // ============ 自动模式梯形速度曲线（T 型）剖面状态 ============
    // 仅由 EtherCAT 实时线程(gradual_approach)读写，故用普通 double，无需加锁；
    // 跨线程的"归零"通过 prof_reset_requested_ 握手，避免丢失更新(Lost Update)
    double prof_v_    = 0.0;   // 当前速度 (脉冲/周期)
    double prof_frac_ = 0.0;   // 位置余数 (脉冲)，整数脉冲下实现平滑低速爬行

    // 剖面归零握手：ROS 回调线程置位(true)，实时线程 exchange(false) 消费（一次性）
    // 置位点：stop() / reset_motion_state() / start_auto_mode() / start_manual_mode()
    //         + 手动模式内各"硬停"出口（限位 Bit11 / 停止请求 / 非使能 / 故障）
    // 禁止在 set_displacement_updated() 等"新指令入口"置位 —— 运动中改目标必须保留速度实现无缝衔接
    std::atomic<bool> prof_reset_requested_{false};

    void gradual_approach(int32_t target_pulses, uint8_t* domain1_pd);

    // ============ 手动点动速度剖面（软起动 / 软停止 / 软换向）============
    // 与 gradual_approach 并列，复用 prof_v_ / prof_frac_ / prof_reset_requested_；
    // 区别：目标是"速度"而非"位置"，用于持续点动，任意时刻改目标都安全
    //   dir      : +1 正转 / -1 反转 / 0 软停止
    //   v_target : 目标巡航速度（工程单位/s，通常即 jog_speed_）
    // 起步 0→±v_target 按 jog_accel 线性爬升；松键 |v|→0 按 jog_decel 递减；
    // 换向先按 jog_decel 减速过零停一拍，再按 jog_accel 反向爬升
    // 仅由 EtherCAT 实时线程调用
    void manual_jog_profile(int dir, double v_target, uint8_t* domain1_pd);

    // 手动点动加减速（工程单位/s²）；0 表示未配置 → manual_jog_profile 按缺省现算
    double jog_accel_ = 0.0;
    double jog_decel_ = 0.0;

    uint16_t current_error_code_ = 0;  // 当前错误代码
};

#endif // SERVO_AXIS_BASE_HPP