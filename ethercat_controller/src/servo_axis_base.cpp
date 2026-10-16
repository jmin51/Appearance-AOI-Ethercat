#include "servo_axis_base.hpp"
#include <algorithm>
#include <iostream>
#include <map>
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
    
    // 梯形速度曲线剖面状态复位
    prof_v_    = 0.0;
    prof_frac_ = 0.0;
    prof_reset_requested_.store(false);

    // 手动点动软起动参数：0 表示未配置，由 manual_jog_profile 按缺省现算
    jog_accel_ = 0.0;
    jog_decel_ = 0.0;

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

    // 清除本次移动速度，避免后续指令误用上一次的百分比速度
    clear_move_speed();

    // 请求实时线程归零速度剖面（直接写 prof_v_ 会与实时线程的读-改-写竞争，丢失更新）
    prof_reset_requested_.store(true, std::memory_order_release);

    std::cout << "轴 " << axis_name_ << " 运动状态已重置: displacement_updated=false, target_pulses=" 
              << target_pulses_ << std::endl;
}

void ServoAxisBase::start_manual_mode() {
    if (current_state_ == AxisState::READY || current_state_ == AxisState::AUTO_MODE) {
        start_manual_requested_ = true;
        // 切换模式意味着放弃当前段：清残留指令标志
        // （否则自动模式下发的点动/位移标志会残留到手动模式，第一周期突然运动）
        clear_pending_motion_requests();
        // 切换模式意味着放弃当前自动段，请求剖面归零
        prof_reset_requested_.store(true, std::memory_order_release);
        std::cout << "轴 " << axis_name_ << " 手动模式启动请求已设置" << std::endl;
    }
}

void ServoAxisBase::start_auto_mode() {
    if (current_state_ == AxisState::READY || current_state_ == AxisState::MANUAL_MODE) {
        start_auto_requested_ = true;
        // 切换模式意味着放弃当前段：清残留指令标志
        // （否则手动模式下发的位移标志会残留到自动模式，第一周期突然跑向旧绝对目标）
        clear_pending_motion_requests();
        // 进入自动模式必须从静止起步：清除手动点动/故障打断可能残留的速度
        prof_reset_requested_.store(true, std::memory_order_release);
        std::cout << "轴 " << axis_name_ << " 自动模式启动请求已设置" << std::endl;
    }
}

void ServoAxisBase::clear_pending_motion_requests() {
    displacement_updated_ = false;
    jog_forward_requested_ = false;
    jog_reverse_requested_ = false;
    jog_stop_requested_ = false;
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
    // 清除本次移动速度，避免复位后残留上次的百分比速度
    clear_move_speed();
    // 请求实时线程归零速度剖面
    prof_reset_requested_.store(true, std::memory_order_release);
    std::cout << "轴 " << axis_name_ << " 运动状态已重置，目标位置设为当前位置 " << joint_position_ << std::endl;
}

// 获取函数实现
std::string ServoAxisBase::get_name() const { return axis_name_; }
AxisState ServoAxisBase::get_current_state() const { return current_state_; }
OperationMode ServoAxisBase::get_operation_mode() const { return operation_mode_; }
// 轴是否“就绪可用”：电平型判定（供上层灯光/联锁/状态判断使用，不参与状态机跳转）
// 语义：READY（已使能、等待选择模式）与 AUTO_MODE/MANUAL_MODE（已切入运行模式）都算就绪。
// 原因：IO 控制模式下轴一旦进入 READY 就会被上层立刻切到 AUTO/MANUAL，
//      若只认 READY 瞬态，上层几乎观察不到“全部轴就绪”，判定结果会随时序偶然变化。
// 显式排除：FAULT / STOPPED / INITIALIZING / UNINITIALIZED。
// 需要“严格等于 READY 瞬态”时（如 IO 模式切换判定），请直接用 get_current_state() == AxisState::READY。
bool ServoAxisBase::is_ready() const {
    // 回原会话（HM）：回原未达标（状态字 bit12 未置位）一律不算就绪。
    // 否则轴一到 0x0633(Switched on) 就会被上层判"系统就绪 → 回原完成"，立即 safe_shutdown 打断回原。
    if (homing_required_.load(std::memory_order_acquire) &&
        !homing_attained_.load(std::memory_order_acquire)) {
        return false;
    }
    return current_state_ == AxisState::READY ||
           current_state_ == AxisState::AUTO_MODE ||
           current_state_ == AxisState::MANUAL_MODE;
}

void ServoAxisBase::begin_homing_session(bool required) {
    // 会话边界 = 每次 configure()（即每次主站重启）：HM 会话要求等回原达标，CSP 会话不参与判定
    homing_required_.store(required, std::memory_order_release);
    homing_attained_.store(false, std::memory_order_release);
    homing_attained_stable_count_ = 0;
    if (required) {
        printf("轴 %s 本次为 HM 回原会话：就绪判定将等待状态字 bit12(回原完成)\n", axis_name_.c_str());
    }
}

void ServoAxisBase::update_homing_attainment(PdsState pds, uint16_t raw_status_word) {
    if (!homing_required_.load(std::memory_order_relaxed)) {
        return;  // 非 HM 会话：bit12 在 CSP 下语义是"驱动器跟随指令"，不参与回原判定
    }
    if (homing_attained_.load(std::memory_order_relaxed)) {
        return;  // 已达标锁存（关机断使能会让 bit12 归零，锁存避免就绪判定抖动）
    }

    // CiA402：HM 下 bit12 = Homing attained；汇川完成瞬间跳到 0x1637（使能 + 到达 + bit12）
    const bool attained_now =
        (pds == PdsState::OPERATION_ENABLED) && ((raw_status_word & 0x1000) != 0);

    if (!attained_now) {
        homing_attained_stable_count_ = 0;
        return;
    }

    if (++homing_attained_stable_count_ >= kHomingAttainedStableCycles) {
        homing_attained_.store(true, std::memory_order_release);
        printf("轴 %s 回原达标：状态字0x%04X 连续 %d 周期为 使能+bit12(回原完成)\n",
               axis_name_.c_str(), raw_status_word, kHomingAttainedStableCycles);
    }
}
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

// ========== CiA402 状态字解码 ==========
// 只取 bit0~3、bit5、bit6 参与判断，bit7(warning) 与 bit8~15(厂商位/回原位) 全部忽略：
//   - bit8~15 会随型号、固件、运行模式(HM 下 bit12=回原完成)变化，不能参与状态跳转
//   - 这样 0x0250 / 0x0650 / 0x1650 / 0x2650 / 0x0670 / 0x0770 统一解码为 SWITCH_ON_DISABLED
PdsState ServoAxisBase::decode_pds_state(uint16_t status_word) {
    const uint16_t mask_4F = 0x004F;  // bits 0,1,2,3,6
    const uint16_t mask_6F = 0x006F;  // bits 0,1,2,3,5,6

    if ((status_word & mask_4F) == 0x0000) return PdsState::NOT_READY;
    if ((status_word & mask_4F) == 0x0040) return PdsState::SWITCH_ON_DISABLED;
    if ((status_word & mask_6F) == 0x0021) return PdsState::READY_TO_SWITCH_ON;
    if ((status_word & mask_6F) == 0x0023) return PdsState::SWITCHED_ON;
    if ((status_word & mask_6F) == 0x0027) return PdsState::OPERATION_ENABLED;
    if ((status_word & mask_6F) == 0x0007) return PdsState::QUICK_STOP_ACTIVE;
    if ((status_word & mask_4F) == 0x000F) return PdsState::FAULT_REACTION_ACTIVE;
    if ((status_word & mask_4F) == 0x0008) return PdsState::FAULT;
    return PdsState::UNKNOWN;
}

const char* ServoAxisBase::pds_state_name(PdsState state) {
    switch (state) {
        case PdsState::NOT_READY:            return "NOT_READY";
        case PdsState::SWITCH_ON_DISABLED:   return "SWITCH_ON_DISABLED";
        case PdsState::READY_TO_SWITCH_ON:   return "READY_TO_SWITCH_ON";
        case PdsState::SWITCHED_ON:          return "SWITCHED_ON";
        case PdsState::OPERATION_ENABLED:    return "OPERATION_ENABLED";
        case PdsState::QUICK_STOP_ACTIVE:    return "QUICK_STOP_ACTIVE";
        case PdsState::FAULT_REACTION_ACTIVE:return "FAULT_REACTION_ACTIVE";
        case PdsState::FAULT:                return "FAULT";
        default:                             return "UNKNOWN";
    }
}

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

// 工程单位 → 脉冲 的换算系数（double，不做整数截断）
// 位移轴: 脉冲/mm ; 角度轴: 脉冲/°
double ServoAxisBase::pulses_per_unit() const {
    const double SCREW_LEAD = 20.0;    // 与 displacement_to_pulses 保持一致
    if (calc_mode_ == AxisCalcMode::ANGLE) {
        return (static_cast<double>(pulses_per_rev_) / 360.0) * gear_ratio_;
    }
    return (static_cast<double>(pulses_per_rev_) / SCREW_LEAD) * gear_ratio_;
}

// ============================================================
// 自动模式梯形速度曲线（T 型）
//   起步 : 速度每周期 +a 从 0 线性爬升   —— 消除"0→巡航"的单周期阶跃
//   巡航 : 达到 v_max 后保持
//   到位 : 按制动曲线 sqrt(2·d·剩余) 提前减速，逐周期递减到 0，精确落位
//   改目标: 保留当前速度平滑转向新目标（无缝衔接，不停顿）
//
// 剖面状态 prof_v_ / prof_frac_ 仅由本函数（EtherCAT 实时线程）读写，
// 跨线程归零通过 prof_reset_requested_ 一次性握手完成，避免丢失更新。
// 已知边界：若新目标落在当前速度的制动距离之内（含方向反转），
//           会以"剩余距离"作为最后一步直接落位，不产生越冲。
// ============================================================
void ServoAxisBase::gradual_approach(int32_t target_pulses, uint8_t* domain1_pd) {
    constexpr double CYCLE_PERIOD = 0.001; // 1ms 周期

    // ① 一次性消费跨线程归零握手（ROS 线程只置位，实际归零在实时线程完成）
    if (prof_reset_requested_.exchange(false, std::memory_order_acquire)) {
        prof_v_    = 0.0;
        prof_frac_ = 0.0;
        // 日志器缓存为函数静态量，避免实时线程里反复构造（本分支仅在停止/复位/切模式时进入）
        static const rclcpp::Logger s_profile_logger = rclcpp::get_logger("servo_profile");
        RCLCPP_DEBUG(s_profile_logger, "轴 %s 速度剖面已归零", axis_name_.c_str());
    }

    const int32_t rem = target_pulses - joint_position_;
    if (rem == 0) {
        // 已到位（正常由子类在 target_pulses_ == joint_position_ 时跳过本函数）
        prof_v_    = 0.0;
        prof_frac_ = 0.0;
        return;
    }

    // ② 脉冲域参数换算
    //    必须用 double 系数：angle_to_pulses()/displacement_to_pulses() 返回 int32_t，
    //    会把加速度（如 0.432 脉冲/周期²）截断成 0，导致梯形曲线完全失效
    const bool use_move_speed = has_move_speed_.load(std::memory_order_acquire);
    const double eff_speed = use_move_speed
                           ? move_speed_.load(std::memory_order_relaxed)
                           : auto_speed_;
    const double ppu   = pulses_per_unit();
    const double v_max = eff_speed * CYCLE_PERIOD * ppu;                    // 脉冲/周期
    const double a_max = accel_ * CYCLE_PERIOD * CYCLE_PERIOD * ppu;        // 脉冲/周期²
    const double d_max = ((decel_ > 0.0) ? decel_ : accel_)
                       * CYCLE_PERIOD * CYCLE_PERIOD * ppu;                 // 脉冲/周期²

    // ③ 收尾：剩余不超过 1 脉冲时直接落位，保证有限周期内收敛
    if (abs(rem) <= 1) {
        joint_position_ = target_pulses;
        prof_v_    = 0.0;
        prof_frac_ = 0.0;
        EC_WRITE_S32(domain1_pd + off_target_position_, joint_position_);
        return;
    }

    // ④ 参数缺失兜底：退回"限速一步式"（无加减速），保证轴仍能运动
    if (v_max <= 0.0 || a_max <= 0.0 || d_max <= 0.0) {
        static std::map<std::string, bool> warned;
        if (!warned[axis_name_]) {
            warned[axis_name_] = true;
            printf("[梯形曲线] 轴 %s 速度/加减速参数非法(speed=%.3f, accel=%.3f, decel=%.3f)，"
                   "已退回限速一步式\n", axis_name_.c_str(), eff_speed, accel_, decel_);
        }
        const int32_t lim = (v_max > 0.0) ? static_cast<int32_t>(v_max) : 1;
        joint_position_ += (abs(rem) > lim) ? ((rem > 0) ? lim : -lim) : rem;
        EC_WRITE_S32(domain1_pd + off_target_position_, joint_position_);
        return;
    }

    const int dir = (rem > 0) ? +1 : -1;

    // ⑤ 制动曲线：剩余行程不足以减速时限制允许速度（"何时开始减速"的唯一判据，每周期重算）
    double v_allow = v_max;
    const double v_brake = std::sqrt(2.0 * d_max * static_cast<double>(abs(rem)));
    if (v_brake < v_allow) {
        v_allow = v_brake;
    }

    // ⑥ 换向必须先归零，避免带着速度反向
    const bool reversing = (prof_v_ != 0.0) && ((prof_v_ > 0.0) != (dir > 0));
    const double v_tgt = reversing ? 0.0 : (dir * v_allow);

    // ⑦ 加速度限幅 —— 这一步生成梯形速度曲线（+a / -d 每周期线性变化）
    if (prof_v_ < v_tgt) {
        prof_v_ = std::min(v_tgt, prof_v_ + a_max);
    } else {
        prof_v_ = std::max(v_tgt, prof_v_ - d_max);
    }

    // ⑧ 位置积分 + 余数累积（整数脉冲必须做，否则低速段会"卡住不动"）
    prof_frac_ += prof_v_;
    int32_t step = static_cast<int32_t>(prof_frac_);
    prof_frac_ -= static_cast<double>(step);
    if (abs(step) > abs(rem)) {          // 不允许越冲
        step = rem;
        prof_frac_ = 0.0;
    }
    joint_position_ += step;
    target_reached_ = false;             // 离开目标位置

    // ⑨ 精确落位后归零速度
    //    子类下一周期会走 else 分支（不再调用本函数），故必须在这里清零，否则残留速度会在
    //    下一次运动时变成起步阶跃
    if (joint_position_ == target_pulses) {
        prof_v_    = 0.0;
        prof_frac_ = 0.0;
    }

    EC_WRITE_S32(domain1_pd + off_target_position_, joint_position_);

    // ⑩ 定期打印速度剖面概览（每 3000 周期 ≈ 3s）
    static std::map<std::string, int> print_counters;
    int& counter = print_counters[axis_name_];
    if (++counter >= 3000) {
        counter = 0;
        const double v_eng = (ppu > 0.0) ? (prof_v_ / (CYCLE_PERIOD * ppu)) : 0.0;
        const double v_lim_eng = (ppu > 0.0) ? (v_max / (CYCLE_PERIOD * ppu)) : 0.0;
        const char* unit = (calc_mode_ == AxisCalcMode::ANGLE) ? "°/s" : "mm/s";
        const char* phase = (v_allow < v_max - 1e-9) ? "减速"
                          : (prof_v_ < v_max - 1e-9) ? "加速" : "巡航";
        printf("[速度剖面] 轴 %s: %s, v=%.3f %s (上限 %.3f), 剩余 %d 脉冲, %s\n",
               axis_name_.c_str(), phase, v_eng, unit, v_lim_eng, rem,
               use_move_speed ? "本次速度" : "默认速度");
    }
}

// ============================================================
// 手动点动速度剖面（软起动 / 软停止 / 软换向）
//   与 gradual_approach 的区别：目标是"速度"而非"位置"，用于持续点动
//   起步 : |v| 每周期 +a 从 0 线性爬升到 jog_speed  —— 消除按下瞬间的阶跃
//   松键 : |v| 每周期 -d 线性递减到 0（dir=0）      —— 软停止，不再回退实际位置
//   换向 : 当前速度与目标反向时先减速过零、停一拍，再反向加速，无冲击
//   半途松键：任意时刻 dir 变化都安全（每周期重算目标速度）；速度越低停得越快
//             （当前速度小于一个周期可减掉的量时，1 个周期直接到 0）
//   参数缺省：jog_accel_ 未配 → 4×v_target；jog_decel_ 未配 → 20×max_jog_speed_
//   剖面状态 prof_v_ / prof_frac_ 仅由本函数（EtherCAT 实时线程）读写，
//   跨线程归零通过 prof_reset_requested_ 一次性握手完成（切模式 / 硬停出口置位）
// ============================================================
void ServoAxisBase::manual_jog_profile(int dir, double v_target, uint8_t* domain1_pd) {
    constexpr double CYCLE_PERIOD = 0.001; // 1ms 周期，与 gradual_approach 保持一致

    // ① 一次性消费跨线程归零握手
    if (prof_reset_requested_.exchange(false, std::memory_order_acquire)) {
        prof_v_    = 0.0;
        prof_frac_ = 0.0;
    }

    const bool stopping = (dir == 0);
    const double ppu = pulses_per_unit();

    // ② 有效加减速（工程单位/s²）：未配置(<=0)时按缺省现算，
    //    保证运行时改 jog_speed / max_jog_speed 后缺省自动跟随
    const double a_eng = (jog_accel_ > 0.0) ? jog_accel_ : 4.0 * v_target;
    const double d_eng = (jog_decel_ > 0.0) ? jog_decel_ : 20.0 * max_jog_speed_;

    // ③ 目标速度与加减速（脉冲域，必须用 double 系数避免 int32 截断）
    const double v_cmd = stopping
                       ? 0.0
                       : ((dir > 0 ? 1.0 : -1.0) * v_target * CYCLE_PERIOD * ppu); // 脉冲/周期
    const double a_max = a_eng * CYCLE_PERIOD * CYCLE_PERIOD * ppu;                // 脉冲/周期²
    const double d_max = d_eng * CYCLE_PERIOD * CYCLE_PERIOD * ppu;                // 脉冲/周期²

    // ④ 参数非法兜底：退回"限速一步式"（直接给目标速度），保证轴仍能运动
    if (a_max <= 0.0 || d_max <= 0.0) {
        static std::map<std::string, bool> warned;
        if (!warned[axis_name_]) {
            warned[axis_name_] = true;
            printf("[手动剖面] 轴 %s 点动加减速参数非法(jog_accel=%.3f, jog_decel=%.3f, v_target=%.3f)，"
                   "已退回限速一步式\n",
                   axis_name_.c_str(), jog_accel_, jog_decel_, v_target);
        }
        prof_v_    = v_cmd;
        prof_frac_ = 0.0;
    } else {
        // ⑤ 换向先归零：当前速度与目标方向相反时本周期只制动，不反向加速
        //    （下一周期速度过零后 reversing 自然失效，转为反向爬升 → 天然"过零停一拍"）
        const bool reversing = (prof_v_ != 0.0) && !stopping && ((prof_v_ > 0.0) != (v_cmd > 0.0));
        const double v_tgt = reversing ? 0.0 : v_cmd;

        // ⑥ 加速度限幅 —— 这一步生成手动梯形速度曲线（+a / -d 每周期线性变化）
        if (prof_v_ < v_tgt) {
            prof_v_ = std::min(v_tgt, prof_v_ + a_max);
        } else {
            prof_v_ = std::max(v_tgt, prof_v_ - d_max);
        }
    }

    // ⑦ 位置积分 + 余数累积（整数脉冲必须做，否则低速段会"卡住不动"）
    prof_frac_ += prof_v_;
    int32_t step = static_cast<int32_t>(prof_frac_);
    prof_frac_ -= static_cast<double>(step);
    joint_position_ += step;
    target_pulses_ = joint_position_;   // 手动模式目标跟随指令位置，兼容其它分支读取

    EC_WRITE_S32(domain1_pd + off_target_position_, joint_position_);

    // ⑧ 停稳后清余数，避免下次起步带入亚脉冲偏移
    if (stopping && prof_v_ == 0.0) {
        prof_frac_ = 0.0;
    }
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

// 速度合法性统一校验：0 < speed <= max_jog_speed_ 且折算到每周期脉冲 >= 1
bool ServoAxisBase::validate_speed(double speed, const char* tag) {
    // 1) 范围校验：上限复用 max_jog_speed_（角度轴 °/s，位移轴 mm/s，单位天然一致）
    if (speed <= 0.0 || speed > max_jog_speed_) {
        std::cout << "轴 " << axis_name_ << " " << tag << "超出范围 (0, "
                  << max_jog_speed_ << "]: " << speed << std::endl;
        return false;
    }

    // 2) 低速保守下限：每周期不足 1 脉冲时，实际运动完全依赖余数累积，单周期位置增量
    //    时有时无、速度极不精确，故仍作为保守下限拒绝（渐变曲线本身能处理 <1 脉冲/周期）
    constexpr double CYCLE_PERIOD = 0.001; // 1ms 周期，与 gradual_approach 保持一致
    const int32_t step = (calc_mode_ == AxisCalcMode::ANGLE)
                       ? angle_to_pulses(speed * CYCLE_PERIOD)
                       : displacement_to_pulses(speed * CYCLE_PERIOD);
    if (step < 1) {
        std::cout << "轴 " << axis_name_ << " " << tag << " (" << speed
                  << ") 折算每周期不足 1 脉冲，速度过低，已拒绝" << std::endl;
        return false;
    }
    return true;
}

// 本次移动速度：设置后仅对紧随其后的一次移动生效，未设置时回落到 auto_speed_
bool ServoAxisBase::set_move_speed(double speed) {
    if (!validate_speed(speed, "本次移动速度")) {
        return false;
    }
    // 先写值, 再置标志(release)，保证实时线程以 acquire 读到标志时速度已就绪
    move_speed_.store(speed, std::memory_order_relaxed);
    has_move_speed_.store(true, std::memory_order_release);
    return true;
}

void ServoAxisBase::clear_move_speed() {
    has_move_speed_.store(false, std::memory_order_release);
}

bool ServoAxisBase::set_auto_speed(double speed) {
    if (!validate_speed(speed, "自动速度")) {
        return false;   // 拒绝，保持原值不变
    }
    auto_speed_ = speed;
    std::cout << "轴 " << axis_name_ << " 自动速度设置为: " << auto_speed_ 
              << (calc_mode_ == AxisCalcMode::ANGLE ? " °/s" : " mm/s") << std::endl;
    return true;
}

// 自动模式梯形速度曲线加减速参数（位移轴 mm/s²、角度轴 °/s²）
// decel <= 0 时取 accel；两者必须为正，否则保留原值并返回 false
bool ServoAxisBase::set_auto_profile(double accel, double decel) {
    if (accel <= 0.0) {
        std::cout << "轴 " << axis_name_ << " 加速度必须为正: " << accel << std::endl;
        return false;
    }
    if (decel <= 0.0) {
        decel = accel;
    }
    accel_ = accel;
    decel_ = decel;
    std::cout << "轴 " << axis_name_ << " 梯形曲线参数: accel=" << accel_
              << ", decel=" << decel_
              << (calc_mode_ == AxisCalcMode::ANGLE ? " °/s²" : " mm/s²") << std::endl;
    return true;
}

// 手动点动速度曲线加减速参数（位移轴 mm/s²、角度轴 °/s²）
// 传 0 表示该项沿用缺省；负值非法被忽略并告警，返回 false
bool ServoAxisBase::set_jog_profile(double accel, double decel) {
    bool ok = true;

    if (accel > 0.0) {
        jog_accel_ = accel;
    } else if (accel != 0.0) {
        std::cout << "轴 " << axis_name_ << " 点动加速度必须为正: " << accel
                  << "，已忽略并沿用缺省(4×点动速度)" << std::endl;
        ok = false;
    }

    if (decel > 0.0) {
        jog_decel_ = decel;
    } else if (decel != 0.0) {
        std::cout << "轴 " << axis_name_ << " 点动减速度必须为正: " << decel
                  << "，已忽略并沿用缺省(20×最大点动速度)" << std::endl;
        ok = false;
    }

    if (ok) {
        std::cout << "轴 " << axis_name_ << " 点动软起动参数: jog_accel="
                  << (jog_accel_ > 0.0 ? jog_accel_ : 4.0 * jog_speed_)
                  << ", jog_decel="
                  << (jog_decel_ > 0.0 ? jog_decel_ : 20.0 * max_jog_speed_)
                  << (calc_mode_ == AxisCalcMode::ANGLE ? " °/s²" : " mm/s²")
                  << " (0 表示缺省)" << std::endl;
    }
    return ok;
}

// ========== 角度软限位实现 ==========
void ServoAxisBase::set_angle_limits(double min_deg, double max_deg) {
    if (min_deg >= max_deg) {
        std::cout << "轴 " << axis_name_ << " 角度限位无效（min=" << min_deg 
                  << " >= max=" << max_deg << "），已忽略" << std::endl;
        has_angle_limits_ = false;
        return;
    }
    min_angle_deg_ = min_deg;
    max_angle_deg_ = max_deg;
    has_angle_limits_ = true;
    std::cout << "轴 " << axis_name_ << " 角度软限位已设置: [" 
              << min_angle_deg_ << "°, " << max_angle_deg_ << "°]" << std::endl;
}

double ServoAxisBase::clamp_angle(double angle_deg) const {
    if (!has_angle_limits_) return angle_deg;
    if (angle_deg < min_angle_deg_) return min_angle_deg_;
    if (angle_deg > max_angle_deg_) return max_angle_deg_;
    return angle_deg;
}