// lights_controller.hpp
// 灯光控制器 - 管理按钮灯和三色灯
// ============================================================================

#ifndef LIGHTS_CONTROLLER_HPP
#define LIGHTS_CONTROLLER_HPP

#include <atomic>
#include "io_interface.hpp"

// ============================================================================
// 三色灯状态
// ============================================================================
enum TricolorLightState {
    LIGHT_OFF,          // 全部熄灭
    LIGHT_RED_ON,       // 红灯常亮（暂停/急停）
    LIGHT_YELLOW_BLINK, // 黄灯闪烁（复位中）
    LIGHT_GREEN_BLINK,  // 绿灯闪烁（就绪）
    LIGHT_GREEN_ON      // 绿灯常亮（运行中）
};

// ============================================================================
// 接口函数
// ============================================================================

// 初始化灯光控制器
void init_lights_controller();

// 更新按钮灯（在main循环中调用）
// 复位按钮需长按确认（时长 = yaml 的 do_module.reset_confirm_ms，缺省 3000ms）
// 急停极性由 yaml 的 di_module.emergency_stop_active_low 决定（缺省低电平有效）
// 灯控不再硬编码DO地址：蜂鸣器/三色灯/按钮灯均通过 do_roles 角色表定位，
// 具体位偏移见 io_config.yaml（不同机型可完全不同）
void update_button_lights(bool start_btn, bool reset_btn, bool pause_btn, 
                          bool emergency_stop = false);

// 更新三色灯和蜂鸣器（在main循环中调用）
void update_tricolor_lights();

// 通知系统就绪（AL states 0x08 且全部轴自动模式）
void notify_system_ready();

// 通知所有轴就绪（启动完成后，绿灯闪烁→常亮）
void notify_all_axes_ready();

// 暂停态灯效（由 main.cpp 在"暂停 toggle"翻转 g_paused 后调用）
//   paused=true ：红灯常亮 + 暂停灯亮（复用红灯；暂停期间轴仍为 READY，
//                 但 notify_system_ready/notify_all_axes_ready 对红灯是 no-op，不会被覆盖）
//   paused=false：暂停灯灭、**启动灯亮**（运行中常态），三色灯落到"绿灯闪烁"，
//                 随后由就绪判定升级为绿灯常亮
//                 （不能设 LIGHT_OFF —— notify_* 只认黄闪/绿闪，OFF 永远升不上去）
void update_paused_lights(bool paused);

// ============================================================================
// 虚拟按钮（方案一：统一事件入口）
// 虚拟按钮只设置虚拟电平，由主循环与实体DI电平合并后统一进入本状态机，
// 保证虚拟/实体按钮的灯效与系统行为完全一致
// ============================================================================

// 虚拟按钮ID（虚拟急停为独立锁存切换，不在此列）
enum VirtualButtonId {
    VBTN_START = 0,   // 启动
    VBTN_RESET,       // 复位（需按住3秒确认）
    VBTN_PAUSE,       // 暂停
    VBTN_COUNT
};

// 设置虚拟按钮电平（按下=true / 释放=false，与实体按钮按压语义一致）
void set_virtual_button(VirtualButtonId id, bool pressed);

// 获取虚拟按钮有效电平（含单周期锁存，极快点击也不会丢失边沿）
bool get_virtual_button(VirtualButtonId id);

// 消费单周期锁存（主循环在update_button_lights之后调用一次）
void consume_virtual_button_latches();

// 虚拟急停锁存切换（再按一次解除）
void toggle_virtual_estop();

// 虚拟急停锁存状态（true=激活）
bool is_virtual_estop_active();

#endif // LIGHTS_CONTROLLER_HPP
