// lights_controller.cpp
// 灯光控制器实现 - 管理按钮灯和三色灯
// ============================================================================

#include "lights_controller.hpp"
#include <cstdio>
#include <chrono>

// ============================================================================
// 内部状态变量
// ============================================================================

// 按钮灯状态
static bool g_start_light_on = false;
static bool g_reset_light_on = false;
static bool g_pause_light_on = false;

// 按钮上次状态（用于边沿检测）
static bool g_last_start_button = false;
static bool g_last_reset_button = false;
static bool g_last_pause_button = false;

// ============================================================================
// 灯控不再硬编码 DO 地址：全部通过角色（do_roles）→ 位偏移写入。
// 不同机型灯位顺序的差异（如老机型 启动灯=位0、圣白 蜂鸣器=位0）全部由
// io_config.yaml 的 do_roles 吸收，本文件不感知具体地址。
// ============================================================================

// 按钮灯上次写入值（用于变化检测）
static bool g_last_start_light_do = false;
static bool g_last_reset_light_do = false;
static bool g_last_pause_light_do = false;

// 复位按钮长按确认计时（确认时长由 yaml 的 do_module.reset_confirm_ms 配置）
static bool g_reset_button_held = false;
static auto g_reset_press_start_time = std::chrono::steady_clock::now();

// 三色灯状态
static TricolorLightState g_tricolor_state = LIGHT_OFF;

// 待启动态标记（复位/回原完成、轴已就绪，但尚未按启动按钮确认）
// 语义：回原达标后“逻辑上等效按下急停按钮”——触发一次完整关闭请求，
//       系统停止、驱动器失能、EtherCAT 主站释放；灯语为 绿灯闪烁 + 暂停灯常亮 + 启动灯闪烁。
//       只有启动按钮能清除（清除后由主循环就绪判定放行“绿灯常亮”）。
// 注意：该状态跨越“系统停止”，不可按 g_system_running==false 清除
static bool g_await_start = false;

// 闪烁控制
static auto g_last_toggle_time = std::chrono::steady_clock::now();
static bool g_blink_state = false;
static bool g_last_red_do = false;
static bool g_last_yellow_do = false;
static bool g_last_green_do = false;
static bool g_last_buzzer_do = false;

// 蜂鸣器间歇节拍基准（启动暖机 / 复位回原共用，占空比 3/10）：
//   以"进入当前提示阶段"的时刻为零点取模，阶段一变就重置（见 update_tricolor_lights()），
//   保证新阶段第一声立即响，不必等一个"停"窗口。
static auto g_buzzer_epoch = std::chrono::steady_clock::now();
static TricolorLightState g_buzzer_last_state = LIGHT_OFF;
static bool g_buzzer_last_await_start = false;

// 闪烁周期(ms)（三色灯/按钮灯用，保持 50% 占空比：500ms 亮 / 500ms 灭）
static const int BLINK_PERIOD_MS = 500;

// 蜂鸣器间歇节拍：周期 1000ms，其中前 300ms 响 → 300ms 响 / 700ms 停（占空比 3/10）
static const int BUZZER_PERIOD_MS = 1000;
static const int BUZZER_ON_MS     = 300;

// ============================================================================
// 外部全局变量声明
// ============================================================================

extern std::atomic<bool> g_system_running;                // 系统运行状态
extern std::atomic<bool> g_start_button_pressed;
extern std::atomic<bool> g_reset_button_pressed;
extern std::atomic<bool> g_pause_toggle_requested;        // 暂停按钮上升沿请求（物理+虚拟）
extern std::atomic<bool> g_paused;                        // 暂停锁存（true=暂停中）
extern std::atomic<int>  g_pending_mode;                  // 暂停期间话题预选的模式（0=无,1=手动,2=自动）
extern std::atomic<bool> g_full_shutdown_requested;       // 完整关闭请求
extern std::atomic<bool> g_reset_pending_after_estop;     // 复位触发的急停后待执行回原

// ============================================================================
// 内部辅助函数
// ============================================================================

// 按角色写入按钮灯 DO（带变化检测）
static void write_button_light(DoRole role, bool state, bool& last_state) {
    if (state != last_state) {
        write_do_role(role, state);
        last_state = state;
    }
}

// ============================================================================
// 接口实现
// ============================================================================

void init_lights_controller() {
    g_start_light_on = false;
    g_reset_light_on = false;
    g_pause_light_on = false;
    g_tricolor_state = LIGHT_OFF;
    g_blink_state = false;
    g_await_start = false;
    g_buzzer_epoch = std::chrono::steady_clock::now();
    g_buzzer_last_state = LIGHT_OFF;
    g_buzzer_last_await_start = false;
    
    // 初始化DO输出（按角色写，未配置的角色会被静默跳过）
    write_do_role(DO_ROLE_BUZZER, false);             // 蜂鸣器
    write_do_role(DO_ROLE_RED_LIGHT, false);          // 红灯
    write_do_role(DO_ROLE_YELLOW_LIGHT, false);       // 黄灯
    write_do_role(DO_ROLE_GREEN_LIGHT, false);        // 绿灯
    write_do_role(DO_ROLE_START_BUTTON_LIGHT, false); // 启动灯
    write_do_role(DO_ROLE_RESET_BUTTON_LIGHT, false); // 复位灯
    write_do_role(DO_ROLE_PAUSE_BUTTON_LIGHT, false); // 暂停灯
    
    printf("[LightsController] 灯光控制器初始化完成\n");
}

// 急停按钮上次状态（用于边沿检测）
static bool g_last_emergency_stop = true;  // 初始假设正常（高电平）

void update_button_lights(bool start_btn, bool reset_btn, bool pause_btn, 
                          bool emergency_stop) {
    // 注：待启动态是“复位/回原完成后系统已停止、等待启动按钮”的状态，会跨越 g_system_running=false；
    //     因此这里不能按“系统未运行”清除 g_await_start，
    //     清除时机只有两处：启动按钮（冷启动分支）与控制器初始化。

    // 边沿检测
    bool start_rising = start_btn && !g_last_start_button;
    bool reset_rising = reset_btn && !g_last_reset_button;
    bool pause_rising = pause_btn && !g_last_pause_button;

    // 启动按钮：点亮启动灯，熄灭其他灯，绿灯闪烁+蜂鸣器，等待轴就绪
    // 急停激活时禁止启动（emergency_stop为高电平=正常）
    if (start_rising && emergency_stop) {
        if (g_system_running.load()) {
            // 系统运行中：待启动态下系统已停止（主站已释放），不会进入此分支；
            // 保留 g_await_start 判断仅作防御，避免运行中误按启动时重复重建资源
            if (g_await_start) {
                g_await_start = false;
                g_start_light_on = true;
                g_reset_light_on = false;
                g_pause_light_on = false;
                printf("[Lights] 启动按钮确认（待启动→运行），绿灯转入常亮\n");
            } else if (g_paused.load()) {
                // 运行中 + 暂停：启动按钮 = 恢复请求。
                //   复用同一个"暂停/恢复请求"标志：方向仍由 main 按 g_paused 判定（now_paused = !g_paused），
                //   因此 main 的暂停分支无需改动。灯效由 main 调用 update_paused_lights(false) 统一驱动。
                //   注意：这里不能置 g_start_button_pressed —— 那是"冷启动"专用标志（要求 !g_system_running），
                //   运行中置位会在后续停机时被误当作冷启动。
                g_pause_toggle_requested.store(true);
                printf("[Lights] 启动按钮触发恢复（暂停 → 运行）\n");
            } else {
                printf("[Lights] 启动按钮按下，系统已在运行，忽略重复启动\n");
            }
        } else {
            // 待启动态结束的唯一路径：系统已停止，由启动按钮发起重新初始化
            // （main.cpp 启动分支会重建 EtherCAT 主站并重新初始化轴，不会重新回原）
            g_start_light_on = true;
            g_reset_light_on = false;
            g_pause_light_on = false;
            g_tricolor_state = LIGHT_GREEN_BLINK;  // 绿灯闪烁（等待轴就绪）
            g_await_start = false;                 // 待启动结束，放行后续“绿灯常亮”
            g_start_button_pressed.store(true);
            g_paused.store(false);                  // 冷启动必定非暂停态
            g_pause_toggle_requested.store(false);  // 清掉可能残留的暂停请求
            g_pending_mode.store(0);                // 冷启动清掉暂停期间的预选
            // 取消复位计时（如果正在进行）
            g_reset_button_held = false;
            printf("[Lights] 启动按钮触发，启动灯亮起，绿灯闪烁，蜂鸣器响，等待轴就绪...\n");
        }
    }

    // 复位按钮：按住3秒确认，点亮复位灯，启动黄灯闪烁+蜂鸣器
    // 急停激活时禁止复位
    if (reset_rising && emergency_stop) {
        // 复位按钮刚按下，开始计时
        g_reset_button_held = true;
        g_reset_press_start_time = std::chrono::steady_clock::now();
        printf("[Lights] 复位按钮按下，开始 %d ms 计时...\n", g_reset_confirm_ms);
    } else if (!reset_btn && g_last_reset_button) {
        // 复位按钮释放，如果未满3秒则取消
        if (g_reset_button_held) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - g_reset_press_start_time).count();
            if (elapsed < g_reset_confirm_ms) {
                printf("[Lights] 复位按钮提前释放（%ld ms < %d ms），取消复位\n",
                       elapsed, g_reset_confirm_ms);
            }
            g_reset_button_held = false;
        }
    }
    
    // 检查是否满足3秒确认条件
    if (g_reset_button_held) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - g_reset_press_start_time).count();
        if (elapsed >= g_reset_confirm_ms && !g_reset_light_on) {
            // 长按确认时间到，先触发急停逻辑，再由main.cpp衔接复位回原流程
            g_reset_light_on = true;
            g_start_light_on = false;
            g_pause_light_on = false;
            g_tricolor_state = LIGHT_YELLOW_BLINK;
            g_await_start = true;                    // 复位流程：回原完成且轴就绪后进入“待启动”
            g_full_shutdown_requested.store(true);   // 触发急停关闭流程
            g_reset_pending_after_estop.store(true); // 标记急停后待执行复位回原
            printf("[Lights] 复位按钮确认（长按 %d ms），先触发急停流程，再执行回原\n",
                   g_reset_confirm_ms);
        }
    }
    
    // 暂停按钮：**单向** —— 只在"非暂停"时请求进入暂停；暂停中的恢复改由启动按钮负责。
    //   两条恢复路径会互相打架，所以暂停中必须忽略本按钮（否则语义退化成 toggle）。
    //   灯效仍由 main 调用 update_paused_lights() 统一驱动，避免"按下即红灯"与实际翻转不一致。
    //   虚拟暂停(VBTN_PAUSE)在 main 里与此合并为同一电平，虚拟按钮同样受此单向约束。
    if (pause_rising && !g_paused.load()) {
        g_pause_toggle_requested.store(true);   // 仅"进入暂停"请求（方向仍由 main 按 g_paused 判定）
        // 取消复位计时（如果正在进行）
        g_reset_button_held = false;
        printf("[Lights] 暂停按钮触发（进入暂停请求）\n");
    } else if (pause_rising) {
        printf("[Lights] 已处于暂停态，忽略暂停按钮（请按启动按钮恢复）\n");
    }

    // ============================================================================
    // 急停按钮：极性由 di_module.emergency_stop_active_low 决定（缺省低电平触发），
    // 所在位由 di_roles.emergency_stop 给出
    // ============================================================================
    // 边沿检测：只在急停状态变化时处理
    bool emergency_active = !emergency_stop;
    bool emergency_rising = emergency_active && !g_last_emergency_stop;
    
    if (emergency_active) {
        g_pause_light_on = true;
        g_start_light_on = false;
        g_reset_light_on = false;
        g_tricolor_state = LIGHT_RED_ON;  // 红灯亮
        // 只在上升沿触发暂停标志和关闭请求，避免每轮循环重复置位
        if (emergency_rising) {
            g_full_shutdown_requested.store(true);   // 触发完整关闭流程（急停只走这一条路径）
            printf("[Lights] 急停激活（emergency_stop=%d），系统进入安全关闭流程，红灯亮\n",
                   emergency_stop);
        }
        // 取消复位计时（如果正在进行）
        g_reset_button_held = false;
        // 物理急停激活时，取消复位回原流程
        g_reset_pending_after_estop.store(false);
    }
    g_last_emergency_stop = emergency_active;
    
    // 更新上次状态
    g_last_start_button = start_btn;
    g_last_reset_button = reset_btn;
    g_last_pause_button = pause_btn;
    
    // 待启动态叠加（仅在绿灯闪烁态生效，不覆盖黄闪/红灯等高优先级状态）：
    //   复位回原完成且轴就绪后（notify_system_ready 已把黄闪转为绿闪），系统随即关闭，
    //   三色灯保持绿闪、暂停灯常亮、启动灯按 500ms 同周期闪烁，提示操作员按启动按钮。
    //   此处不要求 g_system_running：待启动本身就是“系统已停止”的状态
    if (g_await_start && g_tricolor_state == LIGHT_GREEN_BLINK && !emergency_active) {
        g_start_light_on = g_blink_state;
        g_reset_light_on = false;
        g_pause_light_on = true;
    }
    
    // 写入按钮灯DO（只在状态变化时写入）
    write_button_light(DO_ROLE_START_BUTTON_LIGHT, g_start_light_on, g_last_start_light_do);
    write_button_light(DO_ROLE_RESET_BUTTON_LIGHT, g_reset_light_on, g_last_reset_light_do);
    write_button_light(DO_ROLE_PAUSE_BUTTON_LIGHT, g_pause_light_on, g_last_pause_light_do);
}

void update_tricolor_lights() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - g_last_toggle_time).count();
    
    // 每周期切换一次闪烁状态（三色灯/按钮灯用，保持 50% 占空比）
    if (elapsed >= BLINK_PERIOD_MS) {
        g_blink_state = !g_blink_state;
        g_last_toggle_time = now;
    }

    // 蜂鸣器间歇节拍：阶段（三色灯状态 或 待启动标记）变化时把基准重置到当前时刻，
    // 使新阶段第一声立即响（否则可能落在"停"窗口里，最长等 700ms 才响，易被误判为无响应）。
    if (g_tricolor_state != g_buzzer_last_state ||
        g_await_start != g_buzzer_last_await_start) {
        g_buzzer_epoch = now;
        g_buzzer_last_state = g_tricolor_state;
        g_buzzer_last_await_start = g_await_start;
    }
    const auto beep_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - g_buzzer_epoch).count();
    // 每 1000ms 一个周期，前 300ms 响 → 300ms 响 / 700ms 停（占空比 3/10）
    const bool buzzer_periodic_on = ((beep_elapsed % BUZZER_PERIOD_MS) < BUZZER_ON_MS);
    
    // 根据当前状态控制三色灯和蜂鸣器
    bool red_should_on = false;
    bool yellow_should_on = false;
    bool green_should_on = false;
    bool buzzer_should_on = false;  // 蜂鸣器
    
    switch (g_tricolor_state) {
        case LIGHT_RED_ON:
            red_should_on = true;      // 红灯常亮
            yellow_should_on = false;
            green_should_on = false;
            buzzer_should_on = false;  // 红灯时不响蜂鸣器
            break;
        case LIGHT_YELLOW_BLINK:
            red_should_on = false;
            yellow_should_on = g_blink_state;
            green_should_on = false;
            // 蜂鸣器间歇响（3/10：3000ms 周期内响 900ms；不再与黄灯同步）
            buzzer_should_on = buzzer_periodic_on;
            break;
        case LIGHT_GREEN_BLINK:
            red_should_on = false;
            yellow_should_on = false;
            green_should_on = g_blink_state;
            // 启动流程的绿闪（暖机/等待轴就绪）蜂鸣器间歇响（3/10）；
            // 待启动态的绿闪仅作提示，蜂鸣器停
            buzzer_should_on = !g_await_start && buzzer_periodic_on;
            break;
        case LIGHT_GREEN_ON:
            red_should_on = false;
            yellow_should_on = false;
            green_should_on = true;  // 绿灯常亮
            buzzer_should_on = false;
            break;
        case LIGHT_OFF:
        default:
            red_should_on = false;
            yellow_should_on = false;
            green_should_on = false;
            buzzer_should_on = false;
            break;
    }
    
    // 只在状态变化时写入DO（减少Modbus通信）
    if (red_should_on != g_last_red_do) {
        write_do_role(DO_ROLE_RED_LIGHT, red_should_on);
        g_last_red_do = red_should_on;
    }
    if (yellow_should_on != g_last_yellow_do) {
        write_do_role(DO_ROLE_YELLOW_LIGHT, yellow_should_on);
        g_last_yellow_do = yellow_should_on;
    }
    if (green_should_on != g_last_green_do) {
        write_do_role(DO_ROLE_GREEN_LIGHT, green_should_on);
        g_last_green_do = green_should_on;
    }
    if (buzzer_should_on != g_last_buzzer_do) {
        write_do_role(DO_ROLE_BUZZER, buzzer_should_on);
        g_last_buzzer_do = buzzer_should_on;
    }
}

void notify_system_ready() {
    // 从黄灯闪烁切换到绿灯闪烁，停止蜂鸣器
    if (g_tricolor_state == LIGHT_YELLOW_BLINK) {
        g_tricolor_state = LIGHT_GREEN_BLINK;
        if (g_reset_light_on) {
            g_reset_light_on = false;
            write_button_light(DO_ROLE_RESET_BUTTON_LIGHT, false, g_last_reset_light_do);
        }
        // 确保蜂鸣器停止
        if (g_last_buzzer_do) {
            write_do_role(DO_ROLE_BUZZER, false);
            g_last_buzzer_do = false;
        }
        printf("[Lights] 系统就绪，黄灯停闪，绿灯闪烁，蜂鸣器停\n");

        // 复位/回原流程：回原达标（轴就绪、总线 OP）即进入“待启动”，
        // 逻辑上等效按下急停按钮 —— 触发一次完整关闭请求，由 main.cpp 既有的
        // g_full_shutdown_requested 处理分支执行 safe_shutdown(true)：
        // 禁用驱动器、释放 EtherCAT 主站、停止实时线程，系统停在“等待启动按钮”状态。
        // 之后按启动按钮走既有启动分支重新初始化（不会重新回原，零点由驱动器保留）。
        // 注意 1：必须先清 g_reset_pending_after_estop，否则 main 会再衔接一次回原流程。
        // 注意 2：只置 g_full_shutdown_requested —— 急停/复位一律只走这一条路径；
        //         暂停按钮走独立的 g_pause_toggle_requested / g_paused（见 main 暂停 toggle 分支），
        //         两者互不干扰，不再有"两条路径互相抢标志"的问题。
        if (g_await_start) {
            g_reset_pending_after_estop.store(false);
            g_full_shutdown_requested.store(true);
            printf("[Lights] 回原完成，进入待启动：触发系统关闭，等待启动按钮\n");
        }
    }
    // 注意：红灯状态只能通过启动按钮退出，不在此处自动切换
}

void notify_all_axes_ready() {
    // 待启动态（复位回原完成、尚未按启动按钮确认）：不放行绿灯常亮，保持绿闪等待
    if (g_await_start) {
        return;
    }
    // 从绿灯闪烁切换到绿灯常亮（启动完成后）
    if (g_tricolor_state == LIGHT_GREEN_BLINK) {
        g_tricolor_state = LIGHT_GREEN_ON;
        // 确保蜂鸣器停止
        if (g_last_buzzer_do) {
            write_do_role(DO_ROLE_BUZZER, false);
            g_last_buzzer_do = false;
        }
        printf("[Lights] 所有轴就绪，绿灯常亮，蜂鸣器停\n");
    }
}

// 暂停态灯效：由 main.cpp 在"暂停 toggle"翻转 g_paused 后调用
void update_paused_lights(bool paused) {
    if (paused) {
        // 复用红灯常亮（与急停同灯态）。暂停期间轴仍是 READY，
        // 但 notify_system_ready/notify_all_axes_ready 对红灯是 no-op，不会被覆盖。
        g_tricolor_state = LIGHT_RED_ON;
        g_pause_light_on = true;
        g_start_light_on = false;
        g_reset_light_on = false;
        printf("[Lights] 进入暂停态：红灯常亮 + 暂停灯亮\n");
    } else {
        // 恢复运行：
        //   暂停灯灭；启动灯重新点亮（运行中常态，暂停时被本函数灭掉了，必须点回来）；
        //   复位灯保持熄灭；三色灯落到"绿灯闪烁"，由 main 的就绪判定升级为绿灯常亮
        //   （不能设 LIGHT_OFF —— notify_* 只认黄闪/绿闪，OFF 状态永远升不上去）。
        g_pause_light_on = false;
        g_start_light_on = true;
        g_reset_light_on = false;
        g_tricolor_state = LIGHT_GREEN_BLINK;
        printf("[Lights] 解除暂停：暂停灯灭、启动灯亮，三色灯转绿灯闪烁\n");
    }
}

// ============================================================================
// 虚拟按钮状态（方案一：统一事件入口）
// 仅主线程访问（handle_virtual_io回调与主循环均在主线程），无需加锁
// ============================================================================
static bool g_vbtn_level[VBTN_COUNT] = {false};  // 当前电平（按下=true）
static bool g_vbtn_latch[VBTN_COUNT] = {false};  // 按下单周期锁存（极快点击防丢边沿）
static bool g_virtual_estop_active = false;       // 虚拟急停锁存

void set_virtual_button(VirtualButtonId id, bool pressed) {
    if (id >= VBTN_COUNT) return;
    if (pressed) {
        g_vbtn_level[id] = true;
        g_vbtn_latch[id] = true;  // 至少保持到主循环消费一次
    } else {
        g_vbtn_level[id] = false;
    }
}

bool get_virtual_button(VirtualButtonId id) {
    if (id >= VBTN_COUNT) return false;
    return g_vbtn_level[id] || g_vbtn_latch[id];
}

void consume_virtual_button_latches() {
    for (int i = 0; i < VBTN_COUNT; i++) {
        g_vbtn_latch[i] = false;
    }
}

void toggle_virtual_estop() {
    g_virtual_estop_active = !g_virtual_estop_active;
}

bool is_virtual_estop_active() {
    return g_virtual_estop_active;
}
