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
// 复位按钮需要按住3秒才触发
// 急停按钮（M516）低电平触发暂停
void update_button_lights(bool start_btn, bool reset_btn, bool pause_btn, 
                          bool emergency_stop = false);

// 更新三色灯和蜂鸣器（在main循环中调用）
void update_tricolor_lights();

// 设置三色灯状态
void set_tricolor_state(TricolorLightState state);

// 获取当前三色灯状态
TricolorLightState get_tricolor_state();

// 通知系统就绪（AL states 0x08 且全部轴自动模式）
void notify_system_ready();

// 通知所有轴就绪（启动完成后，绿灯闪烁→常亮）
void notify_all_axes_ready();

// 获取按钮灯状态（用于外部查询）
bool is_start_light_on();
bool is_reset_light_on();
bool is_pause_light_on();

#endif // LIGHTS_CONTROLLER_HPP
