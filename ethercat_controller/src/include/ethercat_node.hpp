#ifndef ETHERCAT_NODE_HPP
#define ETHERCAT_NODE_HPP

#include "servo_axis_base.hpp"
#include "servo_axis_factory.hpp"
#include "io_interface.hpp"
#include "fault_management_system.hpp"
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/empty.hpp> // 添加这行，用于Empty消息类型
#include <ecrt.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <atomic>
#include <vector>
#include <memory>

// 应用参数
#define FREQUENCY 1000
#define PERIOD (1.0 / FREQUENCY)
#define CLOCK_TO_USE CLOCK_MONOTONIC

#define NSEC_PER_SEC (1000000000L)
#define PERIOD_NS (NSEC_PER_SEC / FREQUENCY)

// 控制命令定义
#define CMD_START_MANUAL "start_manual"
#define CMD_START_AUTO "start_auto"
#define CMD_STOP "stop"
#define CMD_CLEAR_FAULT "clear_fault"
#define CMD_RESET "reset"

// 扩展指令：轴名:目标值[:百分比]
// percent 仅在 need_percent=true 时有效，取值 5~100（纯整数，如 70）
struct AxisCmdEx {
    std::string name;     // 轴名（与 axes_config.yaml 的 name 一致）
    double      value;    // 目标值（位移轴: mm；角度轴: rad）
    int32_t     percent;  // 速度百分比（相对该轴 max_jog_speed_）
};

class EthercatNode : public rclcpp::Node
{
public:
    EthercatNode(std::string name);
    virtual ~EthercatNode();

    // 轴管理
    void init_axes(ec_master_t *master);
    void register_pdo_entries(ec_domain_t *domain1);
    void handle_axes_state_machines(uint8_t *domain1_pd);
    std::vector<std::shared_ptr<ServoAxisBase>> &get_servo_axes();

    // 配置管理
    std::vector<ec_slave_config_t *> get_all_slave_configs();
    bool load_axes_from_yaml(const std::string &yaml_path); // 新增：从YAML加载轴配置

    // 工具函数
    void publish_joint_states();
    void handle_control_command(const std::string &command);

    // 添加延迟初始化方法
    void initialize_after_axes();

    // IO模块相关
    void start_io_monitoring();
    void stop_io_monitoring();
    // IO 初始化失败的统一出口：配置非法或设备连不上 → 打印 → 置退出标志 → 关闭 ROS
    // （属部署错误，必须 fail fast，避免"IO 全是假数据"导致系统假死且难定位）
    void fail_fast_io(const std::string &reason);
    void handle_io_signals(DI_Interface di);
    bool is_io_running() const { return io_running_.load(); }



    void print_warning(const std::string &message);
    void print_error(const std::string &message);

private:
    void initialize_node();
    void handle_displacement_command(const std_msgs::msg::String::SharedPtr msg);

    // 添加新的解析方法
    bool parse_displacement_command(const std::string &command,
                                    std::vector<std::pair<std::string, double>> &axis_commands);

    // 通用解析："轴名:值"（need_percent=false）或 "轴名:值:整数百分比"（true）
    // 百分比必须为纯整数且 5<=pct<=100；返回 false 表示整条命令非法（整条拒绝）
    bool parse_command_ex(const std::string &command,
                          std::vector<AxisCmdEx> &axis_commands,
                          bool need_percent);

    void handle_axis_command(size_t axis_index, double newTargetPosition);              // 用默认 auto_speed
    void handle_axis_command(size_t axis_index, double newTargetPosition, double speed); // 用本次指定速度
    double pulses_to_displacement(int32_t pulses, int32_t initial_pulses);
    void publish_io_status(); // 新增：发布IO状态

    // 添加与Python节点通信的方法
    void handle_py_control_command(const std_msgs::msg::String::SharedPtr msg);

    // 添加点动指令处理函数
    void handle_jog_command(const std_msgs::msg::String::SharedPtr msg);

    // 添加虚拟IO指令处理
    void handle_virtual_io(const std_msgs::msg::String::SharedPtr msg);

    std::vector<std::shared_ptr<ServoAxisBase>> servo_axes_;
    std::vector<double> last_target_positions_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr system_status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr io_status_pub_;       // 新增IO状态发布器
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr displacement_sub_; // 从Float64MultiArray改为String
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_command_sub_;

    // 添加角度控制订阅器 (rad)
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr angle_sub_;
    void handle_angle_command(const std_msgs::msg::String::SharedPtr msg);
    double rad_to_mm(double rad);  // 弧度转毫米

    // 速度百分比指令订阅器（位移轴 / 角度轴各一，消息格式 "轴名:值:百分比"）
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr displacement_pct_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr angle_pct_sub_;
    void handle_displacement_pct_command(const std_msgs::msg::String::SharedPtr msg);
    void handle_angle_pct_command(const std_msgs::msg::String::SharedPtr msg);

    // 添加与Python节点通信的发布器和订阅器
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr py_io_status_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr py_control_command_sub_;

    // 添加点动指令订阅器
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr jog_command_sub_;

    // 添加IO控制话题
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr do_control_sub_;
    // 添加点动速度设置订阅器
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr jog_speed_sub_;
    // 虚拟IO订阅器
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr virtual_io_sub_;

    // 10ms 定时器，用于发布关节状态和检查层移动完成
    rclcpp::TimerBase::SharedPtr periodic_timer_;
    // 定时器回调函数
    void periodic_timer_callback();

    // 添加点动速度设置处理函数
    void handle_jog_speed_command(const std_msgs::msg::String::SharedPtr msg);

    // 添加点动速度解析函数
    bool parse_jog_speed_command(const std::string &command, std::string &axis_name, double &speed);

    std::atomic<bool> node_shutting_down_;

    // 故障管理相关：在现有成员变量中添加
    std::unique_ptr<fault_management::FaultManagementSystem> fault_manager_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fault_code_pub_;

    // IO模块相关
    std::atomic<bool> io_running_{false};
    DI_Interface current_di_status_;
    DO_Interface current_do_control_;
    pthread_mutex_t io_mutex_;

    // 状态变化检测（避免日志洪泛）
    bool last_all_axes_ready_ = false;    // 上次所有轴就绪状态
    bool last_manual_auto_state_ = false; // 上次手自动状态

    // 命令去重防抖（防止重复命令洪泛）
    std::string last_command_;                              // 上次执行的命令
    rclcpp::Time last_command_time_{0, 0, RCL_STEADY_TIME}; // 上次命令时间
    static constexpr double CMD_DEDUP_SEC = 0.5;            // 去重窗口（秒）

    // 周期性系统状态发布
    void publish_system_status();

    void monitor_di_changes(const DI_Interface &current_di);
    void handle_do_control(const std_msgs::msg::String::SharedPtr msg);
    
    bool are_all_axes_in_auto_mode(); // 检查所有轴是否都在自动模式
};

// 全局变量声明
extern std::shared_ptr<EthercatNode> global_node;
extern ec_master_t *master;
extern ec_domain_t *domain1;
extern uint8_t *domain1_pd;
extern std::atomic<bool> g_should_exit;
// 在全局变量定义区域添加
extern std::atomic<bool> g_system_running;       // 系统运行状态
extern std::atomic<bool> g_start_button_pressed; // 启动按钮状态
extern std::atomic<bool> g_pause_toggle_requested; // 暂停按钮上升沿请求（物理+虚拟）
extern std::atomic<bool> g_paused;                 // 暂停锁存（true=暂停中，轴驻留 READY）
extern std::atomic<int>  g_pending_mode;           // 暂停期间话题预选的模式（0=无,1=手动,2=自动）
extern std::atomic<bool> g_reset_button_pressed; // 新增：复位按钮状态

// 全局函数声明
void signal_handler(int signum);
void safe_shutdown(bool is_pause);
void *rt_task_wrapper(void *arg);

#endif // ETHERCAT_NODE_HPP