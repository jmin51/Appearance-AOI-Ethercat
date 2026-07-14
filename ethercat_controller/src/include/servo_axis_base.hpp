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
    virtual bool set_auto_speed(double speed);
    virtual double get_auto_speed() const { return auto_speed_; }

protected:
    // 保护成员变量 - 子类可以访问
    std::string axis_name_;
    uint16_t slave_position_;
    AxisType axis_type_;
    DriveBrand brand_;
    uint32_t product_code_;  // 在基类中存储产品号
    double gear_ratio_; // 添加减速比成员变量
    AxisCalcMode calc_mode_; // 轴计算模式
    
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
    double auto_speed_;                // 自动模式移动速度
    const double DEFAULT_AUTO_SPEED = 5.0; // 默认自动速度
    
    // 在保护成员变量中添加
    std::atomic<bool> target_reached_flag_{false};
    mutable std::mutex flag_mutex_;

    // 新增逐步逼近相关变量
    int32_t target_offset_;           // 目标偏移量
    int direction_flag_;              // 0: 无方向 1: 正转 -1: 反转
    int32_t new_target_;              // 新目标位置
    
    void gradual_approach(int32_t target_pulses, uint8_t* domain1_pd);
    
    // 获取最大步进脉冲数（子类可重写以支持不同限速）
    virtual int32_t get_max_step() const { return 40; }

    uint16_t current_error_code_ = 0;  // 当前错误代码
};

#endif // SERVO_AXIS_BASE_HPP