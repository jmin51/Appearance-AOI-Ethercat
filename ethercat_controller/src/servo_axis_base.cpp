#include "servo_axis_base.hpp"
#include <iostream>
#include <cmath>

ServoAxisBase::ServoAxisBase(const std::string& name, uint16_t position, 
                           AxisType axis_type, DriveBrand brand, uint32_t product_code, double gear_ratio, AxisCalcMode calc_mode)
    : axis_name_(name), slave_position_(position), axis_type_(axis_type), 
      brand_(brand), product_code_(product_code), gear_ratio_(gear_ratio), calc_mode_(calc_mode) {
    initialize_members();
}

void ServoAxisBase::initialize_members() {
    sc_ = nullptr;
    control_word_ = 0;
    status_word_ = 0;
    off_target_position_ = 0;
    off_actual_position_ = 0;
    off_error_code_ = 0;
    
    current_state_ = AxisState::UNINITIALIZED;
    operation_mode_ = OperationMode::AUTO;
    
    position_initialized_ = false;
    target_pulses_ = 0;
    target_displacement_ = 0.0;
    displacement_updated_ = false;
    joint_position_ = 0;
    initial_position_ = 0;
    target_reached_ = 0;
    
    homing_in_progress_ = false;
    homing_completed_ = false;
    
    start_manual_requested_ = false;
    start_auto_requested_ = false;
    stop_requested_ = false;
    clear_fault_requested_ = false;
    reset_requested_ = false;
    fault_clearing_in_progress_ = false;
    fault_clear_step_ = 0;
    fault_clear_counter_ = 0;

    // 添加速度控制初始化
    jog_speed_ = DEFAULT_JOG_SPEED;
    auto_speed_ = DEFAULT_AUTO_SPEED;
    jog_forward_requested_ = false;
    jog_reverse_requested_ = false;
    jog_stop_requested_ = false;
    limit_trigger_direction_ = 0;     // 限位触发方向初始化
    
    // +++ 新增：目标到达标志重置 +++
    target_reached_flag_.store(false);
    target_reached_ = false;
    
    // +++ 新增：逐步逼近相关变量重置 +++
    target_offset_ = 0;
    direction_flag_ = 0;
    new_target_ = 0;
    
    current_error_code_ = 0;
}

// 基类的默认实现
void ServoAxisBase::set_target_displacement(double displacement) {
    target_displacement_ = displacement;
}

void ServoAxisBase::set_displacement_updated(bool updated) {
    displacement_updated_ = updated;
    
    // +++ 关键修复：设置新位移时立即清除完成标志，防止竞态条件导致误报 +++
    if (updated) {
        std::lock_guard<std::mutex> lock(flag_mutex_);
        target_reached_flag_ = false;
        target_reached_ = false;
    }
}

void ServoAxisBase::stop() {
    // 结束作业：先发送停止请求，状态机中处理减速后再跳转
    if (current_state_ == AxisState::MANUAL_MODE || current_state_ == AxisState::AUTO_MODE) {
        stop_requested_ = true;
        // std::cout << "轴 " << axis_name_ << " 停止请求已设置" << std::endl;
    }
    
    // 重置目标位置为当前位置（立即停止运动）
    target_pulses_ = joint_position_;
    
    // 清除点动请求标志
    jog_forward_requested_ = false;
    jog_reverse_requested_ = false;
    jog_stop_requested_ = false;
    
    // 重置目标到达标志
    target_reached_ = false;
    {
        std::lock_guard<std::mutex> lock(flag_mutex_);
        target_reached_flag_ = false;
    }
    
    std::cout << "轴 " << axis_name_ << " 运动状态已重置: displacement_updated=false, target_pulses=" 
              << target_pulses_ << std::endl;
}

void ServoAxisBase::start_manual_mode() {
    if (current_state_ == AxisState::READY || current_state_ == AxisState::AUTO_MODE) {
        start_manual_requested_ = true;
        std::cout << "轴 " << axis_name_ << " 手动模式启动请求已设置" << std::endl;
    }
}

void ServoAxisBase::start_auto_mode() {
    if (current_state_ == AxisState::READY || current_state_ == AxisState::MANUAL_MODE) {
        start_auto_requested_ = true;
        std::cout << "轴 " << axis_name_ << " 自动模式启动请求已设置" << std::endl;
    }
}

void ServoAxisBase::clear_fault() {
    if (current_state_ == AxisState::FAULT) {
        clear_fault_requested_ = true;
        std::cout << "轴 " << axis_name_ << " 故障清除请求已设置" << std::endl;
    }
}

void ServoAxisBase::reset_axis() {
    reset_requested_ = true;
    // 如果当前在故障状态，同时触发故障清除流程
    // 避免reset只是重置软件状态，而硬件故障未被清除
    if (current_state_ == AxisState::FAULT) {
        clear_fault_requested_ = true;
        std::cout << "轴 " << axis_name_ << " 重置请求已设置（同时触发故障清除）" << std::endl;
    } else {
        std::cout << "轴 " << axis_name_ << " 重置请求已设置" << std::endl;
    }
}

void ServoAxisBase::reset_motion_state() {
    // 重置运动状态，避免恢复时误判目标到达
    // 将目标位置设为当前位置，确保恢复后需要新的位移指令才会移动
    target_pulses_ = joint_position_;
    target_displacement_ = 0.0;
    displacement_updated_ = false;
    target_reached_ = false;
    {
        std::lock_guard<std::mutex> lock(flag_mutex_);
        target_reached_flag_ = false;
    }
    // 重置逐步逼近相关变量
    target_offset_ = 0;
    direction_flag_ = 0;
    new_target_ = 0;
    std::cout << "轴 " << axis_name_ << " 运动状态已重置，目标位置设为当前位置 " << joint_position_ << std::endl;
}

// 获取函数实现
std::string ServoAxisBase::get_name() const { return axis_name_; }
AxisState ServoAxisBase::get_current_state() const { return current_state_; }
OperationMode ServoAxisBase::get_operation_mode() const { return operation_mode_; }
bool ServoAxisBase::is_ready() const { return current_state_ == AxisState::READY; }
bool ServoAxisBase::is_running() const { 
    return current_state_ == AxisState::MANUAL_MODE || current_state_ == AxisState::AUTO_MODE; 
}
bool ServoAxisBase::is_homing_completed() const { return homing_completed_; }
bool ServoAxisBase::is_homing_in_progress() const { return homing_in_progress_; }
ec_slave_config_t* ServoAxisBase::get_slave_config() { return sc_; }
unsigned int ServoAxisBase::get_control_word_offset() const { return control_word_; }
int32_t ServoAxisBase::get_actual_position() const { return joint_position_; }
int32_t ServoAxisBase::get_initial_position() const { return initial_position_; }
DriveBrand ServoAxisBase::get_brand() const { return brand_; }

// 保护方法实现
int32_t ServoAxisBase::displacement_to_pulses(double displacement_mm) {
    const double SCREW_LEAD = 20.0;  // 丝杠导程20mm
    
    // 考虑减速比: 实际电机转数 = 位移 / 导程 × 减速比
    double revolutions = (displacement_mm / SCREW_LEAD) * gear_ratio_;
    return static_cast<int32_t>(revolutions * pulses_per_rev_);
}

double ServoAxisBase::pulses_to_displacement(int32_t pulses) {
    const double SCREW_LEAD = 20.0;
    
    double revolutions = static_cast<double>(pulses) / pulses_per_rev_;
    // 考虑减速比: 实际位移 = 电机转数 / 减速比 × 导程
    return (revolutions / gear_ratio_) * SCREW_LEAD;
}

// 新增：角度 -> 脉冲（旋转轴专用）
int32_t ServoAxisBase::angle_to_pulses(double angle_degree)
{
    // 每度脉冲数 = 每转脉冲数 / 360°
    const double pulse_per_degree = static_cast<double>(pulses_per_rev_) / 360.0;   

    // 脉冲 = 角度 * 每度脉冲数 * 减速比
    double total_pulses = angle_degree * pulse_per_degree * gear_ratio_;
    return static_cast<int32_t>(std::round(total_pulses));
}

double ServoAxisBase::pulses_to_angle(int32_t pulses)
 {
    const double pulse_per_degree = static_cast<double>(pulses_per_rev_) / 360.0;   
    
    // 角度 = 脉冲 / 每度脉冲数 / 减速比
    double total_degree = static_cast<double>(pulses) / pulse_per_degree / gear_ratio_;
    return (total_degree);
}

void ServoAxisBase::check_state_changes(uint16_t read_status_word, uint16_t error_code) {
    // 为每个轴单独记录状态字
    static std::map<std::string, uint16_t> last_status_words;
    
    uint16_t& last_status_word = last_status_words[axis_name_];
    
    if (read_status_word != last_status_word) {
        std::cout << "轴 " << axis_name_ << " 状态字变化: 0x" 
                  << std::hex << last_status_word << " -> 0x" << read_status_word << std::dec << std::endl;
        last_status_word = read_status_word;
    }
}

void ServoAxisBase::handle_fault_clear(uint8_t* domain1_pd) {
    fault_clear_counter_++;
    uint16_t current_status = EC_READ_U16(domain1_pd + status_word_);
    
    switch (fault_clear_step_) {
        case 0:
            // 第一步：写0x0080（故障复位）
            EC_WRITE_U16(domain1_pd + control_word_, 0x0080);
            printf("轴 %s 故障清除步骤%d: 发送0x0080\n", axis_name_.c_str(), fault_clear_step_);
            if (fault_clear_counter_ > 10) {
                fault_clear_step_ = 1;
                fault_clear_counter_ = 0;
            }
            break;
            
        case 1:
            // 第二步：写0x0000（准备使能）
            EC_WRITE_U16(domain1_pd + control_word_, 0x0000);
            printf("轴 %s 故障清除步骤%d: 发送0x0000\n", axis_name_.c_str(), fault_clear_step_);
            if (fault_clear_counter_ > 5) {
                fault_clear_step_ = 2;
                fault_clear_counter_ = 0;
            }
            break;
            
        case 2:
            // 第三步：写0x0006（切换到准备开关ON）
            EC_WRITE_U16(domain1_pd + control_word_, 0x0006);
            printf("轴 %s 故障清除步骤%d: 发送0x0006\n", axis_name_.c_str(), fault_clear_step_);
            if (fault_clear_counter_ > 5) {
                fault_clear_step_ = 3;
                fault_clear_counter_ = 0;
            }
            break;
            
        case 3:
            // 第四步：检查状态字，确认故障已清除
            // 故障清除成功的标志：bit3(故障位)为0
            if ((current_status & 0x0008) == 0) {
                // 故障位已清除，说明故障清除成功
                fault_clearing_in_progress_ = false;
                
                // 重置位置信息
                int32_t current_pos = EC_READ_S32(domain1_pd + off_actual_position_);
                target_pulses_ = current_pos;
                joint_position_ = current_pos;
                initial_position_ = current_pos;

                // 关键：清除成功后必须跳到 INITIALIZING，让驱动器重新走完整 0x0006→0x0007→0x000F 序列
                // 否则状态机停留在 FAULT，FAULT 分支仅写 0x0006，驱动器永远停在 0x1638，无法接收新指令
                current_state_ = AxisState::INITIALIZING;

                printf("轴 %s 故障清除完成，状态字: 0x%04x, 跳转到 INITIALIZING\n",
                    axis_name_.c_str(), current_status);
            } else if (fault_clear_counter_ > 100) {
                // 超时处理
                fault_clearing_in_progress_ = false;
                printf("轴 %s 故障清除超时，当前状态字: 0x%04x\n",
                    axis_name_.c_str(), current_status);
            }
            break;
    }
}

void ServoAxisBase::check_system_initialization() {
    // 基类实现系统初始化检查
    if (homing_completed_) {
        std::cout << "轴 " << axis_name_ << " 回零完成，系统可初始化" << std::endl;
    }
}

// 在 ServoAxisBase.cpp 中实现
bool ServoAxisBase::check_target_reached_flag() {
    std::lock_guard<std::mutex> lock(flag_mutex_);
    if (target_reached_flag_) {
        target_reached_flag_ = false; // 读取后清除标志
        return true;
    }
    return false;
}

bool ServoAxisBase::is_target_reached() const {
    std::lock_guard<std::mutex> lock(flag_mutex_);
    return target_reached_flag_;
}

// 在逐步逼近函数中设置标志位
void ServoAxisBase::gradual_approach(int32_t target_pulses, uint8_t* domain1_pd) {
    const int32_t TOLERANCE = 50;
    int32_t error = target_pulses - joint_position_;
    int32_t max_step = 0;  // 声明在函数顶部
    int32_t step = 0;      // 声明在函数顶部
    
    // 速度打印计数器（每1000次打印一次，约1秒）
    static std::map<std::string, int> print_counters;
    static std::map<std::string, int32_t> last_positions;
    int& counter = print_counters[axis_name_];
    int32_t& last_pos = last_positions[axis_name_];
    
    // 根据自动速度计算每周期最大步长（与手动模式速度计算方式一致）
    // 位移轴: auto_speed 单位为 mm/s, 角度轴: auto_speed 单位为 °/s
    constexpr double CYCLE_PERIOD = 0.001; // 1ms 周期
    if (calc_mode_ == AxisCalcMode::ANGLE) {
        max_step = angle_to_pulses(auto_speed_ * CYCLE_PERIOD);
    } else {
        max_step = displacement_to_pulses(auto_speed_ * CYCLE_PERIOD);
    }
    
    if (abs(error) <= TOLERANCE) {
        if (!target_reached_) {
            // 第一次到达目标位置
            std::lock_guard<std::mutex> lock(flag_mutex_);
            target_reached_flag_ = true;
            target_reached_ = true;
            printf("轴 %s 已到达目标位置!\n", axis_name_.c_str());
        }
        joint_position_ = target_pulses; // 精确对齐
    } else {
        step = (abs(error) > max_step) ? 
               ((error > 0) ? max_step : -max_step) : error;
        joint_position_ += step;
        target_reached_ = false; // 离开目标位置
        
        // 定期打印速度信息（每3000个周期约3000ms）
        counter++;
        if (counter >= 3000) {
            counter = 0;
            int32_t pos_diff = abs(joint_position_ - last_pos);
            if (calc_mode_ == AxisCalcMode::ANGLE) {
                // 角度轴: 脉冲 -> °/s
                double speed_deg_per_s = pulses_to_angle(pos_diff);
                printf("[速度监控] 轴 %s: 步长=%d脉冲, 当前速度=%.2f °/s (auto_speed=%.1f °/s, MAX_STEP=%d, error=%d)\n", 
                       axis_name_.c_str(), step, speed_deg_per_s, auto_speed_, max_step, error);
            } else {
                // 位移轴: 脉冲 -> mm/s
                double speed_mm_per_s = pulses_to_displacement(pos_diff);
                printf("[速度监控] 轴 %s: 步长=%d脉冲, 当前速度=%.2f mm/s (auto_speed=%.1f mm/s, MAX_STEP=%d, error=%d)\n", 
                       axis_name_.c_str(), step, speed_mm_per_s, auto_speed_, max_step, error);
            }
            last_pos = joint_position_;
        }
    }
    
    EC_WRITE_S32(domain1_pd + off_target_position_, joint_position_);
}

bool ServoAxisBase::set_jog_speed(double speed) {
    std::lock_guard<std::mutex> lock(speed_mutex_);
    
    // 检查速度范围（由YAML配置的每轴独立范围）
    if (speed < min_jog_speed_ || speed > max_jog_speed_) {
        std::cout << "轴 " << axis_name_ << " 点动速度超出范围 [" 
                  << min_jog_speed_ << ", " << max_jog_speed_ << "]: " << speed << std::endl;
        return false;
    }

    jog_speed_ = speed;
    std::cout << "轴 " << axis_name_ << " 点动速度设置为: " << jog_speed_ << std::endl;
    return true;
}

bool ServoAxisBase::set_auto_speed(double speed) {
    auto_speed_ = speed;
    std::cout << "轴 " << axis_name_ << " 自动速度设置为: " << auto_speed_ 
              << (calc_mode_ == AxisCalcMode::ANGLE ? " °/s" : " mm/s") << std::endl;
    return true;
}