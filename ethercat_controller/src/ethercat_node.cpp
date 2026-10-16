#include "ethercat_node.hpp"
#include "io_modules/lights_controller.hpp" // 灯光控制器
#include <thread>
#include <chrono>
#include <signal.h>
#include <sys/mman.h>
#include <modbus/modbus.h>
#include <string>
#include <sstream>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#define CONTROL_SOURCE_IO 1 // 1:使用IO控制手自动模式, 0:使用话题控制
// 全局变量定义
std::shared_ptr<EthercatNode> global_node = nullptr;
ec_master_t *master = nullptr;
ec_domain_t *domain1 = nullptr;
uint8_t *domain1_pd = nullptr;
std::atomic<bool> g_should_exit(false);
std::atomic<bool> node_shutting_down_{false};
// 全局变量定义
std::atomic<bool> g_system_running(false);
std::atomic<bool> g_start_button_pressed(false);
// 硬件急停电平（true=急停按下，由 main.cpp 主循环按 DI 极性归一化后写入）
// 与虚拟急停软件锁存 g_virtual_estop_active 严格独立，二者互不写入（见 globals.h）
std::atomic<bool> g_hw_estop_active(false);
// 按流程停机锁存（急停/复位回原/待启动）：用于 /system_status 基态上报 STOPPED
std::atomic<bool> g_system_halted(false);
// "暂停/恢复请求"（物理暂停 DI pause_button 与 Qt 虚拟暂停 VBTN_PAUSE 合并后触发）。
// 触发源有两个，方向由 main 按 g_paused 取反判定（now_paused = !g_paused），main 无需区分来源：
//   1) 非暂停态按下暂停按钮（上升沿）→ 请求进入暂停；
//   2) 运行中且处于暂停态按下启动按钮（上升沿）→ 请求恢复（暂停中的暂停按钮被灯控忽略）。
// 单独一个标志是为了不再复用"急停/复位共用"的语义（历史上二者混用会互相踩）。
std::atomic<bool> g_pause_toggle_requested(false);
// 暂停锁存：true=暂停中。暂停时轴被 stop() 回 READY 并驻留，IO 控模式不自动进模式；
// **恢复由启动按钮触发**，恢复时按"话题预选 > DI 手自动位 > 默认模式"进对应模式。
// 语义见 main.cpp 主循环的暂停分支。
std::atomic<bool> g_paused(false);
// 暂停期间"话题预选"的模式：0=未预选、1=手动、2=自动。
// 暂停中收到 /control_command 的 start_manual/start_auto 只记录到这里（不立即切模式，
// 否则暂停会被打穿：轴进模式后 is_running() 为真，位移指令就会执行）。
// 解除暂停时优先用它（一次性，exchange(0) 读走即清），无预选才回落到 DI 手自动位/默认模式。
std::atomic<int> g_pending_mode(0);
std::atomic<bool> g_reset_button_pressed(false); // 复位按钮状态
std::atomic<bool> g_full_shutdown_requested(false);       // 完整关闭请求（急停/复位触发）
std::atomic<bool> g_reset_pending_after_estop(false);     // 复位触发的急停后待执行回原

// 添加缺失的常量定义
bool running = true;  // 添加缺失的running变量

// 其他全局变量
ec_master_state_t master_state = {};
ec_domain_state_t domain1_state = {};
unsigned int counter = 0;
unsigned int blink = 0;
unsigned int sync_ref_counter = 0;
// cycletime 在 main.cpp 中定义
pthread_t thread = 0;

std::atomic<bool> homing_completed{false};

EthercatNode::EthercatNode(std::string name) : Node(name)
{
    initialize_node();
}

EthercatNode::~EthercatNode()
{
    node_shutting_down_.store(true);
    stop_io_monitoring(); // 停止IO监控
}

void EthercatNode::initialize_node()
{
    RCLCPP_INFO(this->get_logger(), "初始化EtherCAT节点: %s", this->get_name());

    // 创建发布器和订阅器
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 50);
    system_status_pub_ = this->create_publisher<std_msgs::msg::String>("/system_status", 10);
    io_status_pub_ = this->create_publisher<std_msgs::msg::String>("/io_status", 10); // 新增IO状态发布器

    // 添加与Python节点通信的发布器和订阅器
    py_io_status_pub_ = this->create_publisher<std_msgs::msg::String>("/py_io_status", 10);
    py_control_command_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/py_control_command", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::String::SharedPtr msg)
        {
            this->handle_py_control_command(msg);
        });

    displacement_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/displacement_command", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::String::SharedPtr msg)
        {
            handle_displacement_command(msg);
        });

    // 速度百分比指令（位移轴）："轴名:目标mm:百分比"
    displacement_pct_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/displacement_command_pct", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::String::SharedPtr msg)
        {
            handle_displacement_pct_command(msg);
        });

    // 添加角度控制订阅器 (rad)
    angle_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/angle_command", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::String::SharedPtr msg)
        {
            handle_angle_command(msg);
        });

    // 速度百分比指令（角度轴）："轴名:目标rad:百分比"
    angle_pct_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/angle_command_pct", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::String::SharedPtr msg)
        {
            handle_angle_pct_command(msg);
        });

    control_command_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/control_command", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::String::SharedPtr msg)
        {
            handle_control_command(msg->data);
        });
    // 添加点动指令订阅
    jog_command_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/jog_command", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::String::SharedPtr msg)
        {
            handle_jog_command(msg);
        });
    // 新增：点动速度设置订阅
    jog_speed_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/jog_speed_command", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::String::SharedPtr msg)
        {
            handle_jog_speed_command(msg);
        });

    // 添加DO控制话题订阅
    do_control_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/do_control", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::String::SharedPtr msg)
        {
            handle_do_control(msg);
        });

    // 添加虚拟IO话题订阅
    virtual_io_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/virtual_io", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::String::SharedPtr msg) {
            this->handle_virtual_io(msg);
        });
    // 添加故障码发布器初始化
    fault_code_pub_ = this->create_publisher<std_msgs::msg::String>("/fault_code", 10);
    // 初始化故障管理器（升级为故障管理系统）
    fault_manager_ = std::make_unique<fault_management::FaultManagementSystem>(this);
    fault_manager_->set_fault_publisher(fault_code_pub_);
    fault_manager_->initialize(); // 启用日志回调自动捕获

    // +++ 新增：创建并启动 100ms 周期定时器 +++
    periodic_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&EthercatNode::periodic_timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "100ms 周期定时器已创建并启动");

    // 初始化IO互斥锁
    pthread_mutex_init(&io_mutex_, nullptr);

    // 声明轴配置文件路径参数（默认从包的share目录读取）
    this->declare_parameter<std::string>(
        "axes_config_path",
        ament_index_cpp::get_package_share_directory("ethercat_controller") + "/config/axes_config.yaml");

    // 声明IO配置文件路径参数
    this->declare_parameter<std::string>(
        "io_config_path",
        ament_index_cpp::get_package_share_directory("ethercat_controller") + "/config/io_config.yaml");

    RCLCPP_INFO(this->get_logger(), "EtherCAT节点初始化完成");
}

// +++ 新增：定时器回调函数实现 +++
void EthercatNode::periodic_timer_callback()
{
    // 添加关闭检查
    if (node_shutting_down_.load() || !rclcpp::ok())
    {
        return;
    }

    // 1. 发布关节状态
    publish_joint_states();

    // 2. 发布故障状态
    if (fault_manager_)
    {
        fault_manager_->publish_fault_status();
    }

    // 3. 发布系统状态 (每 100ms 一次)
    publish_system_status();
}

void EthercatNode::publish_system_status()
{
    if (servo_axes_.empty()) return;

    // 聚合所有轴的状态
    bool all_fault   = true;
    bool all_ready   = true;
    bool all_manual  = true;
    bool all_auto    = true;
    bool all_stopped = true;
    bool any_fault   = false;

    for (auto &axis : servo_axes_)
    {
        AxisState s = axis->get_current_state();
        if (s == AxisState::FAULT)      any_fault = true;
        if (s != AxisState::FAULT)      all_fault   = false;
        if (s != AxisState::READY)      all_ready   = false;
        if (s != AxisState::MANUAL_MODE) all_manual = false;
        if (s != AxisState::AUTO_MODE)  all_auto    = false;
        if (s != AxisState::STOPPED)    all_stopped = false;
    }

    std::string status;
    if (all_fault || any_fault)      status = "FAULT";       // 故障优先，保持可见性
    else if (g_system_halted.load()) status = "STOPPED";     // 按流程停机（急停/复位回原/待启动）
    else if (all_ready)              status = "READY";
    else if (all_manual)             status = "MANUAL";
    else if (all_auto)               status = "AUTO";
    else if (all_stopped)            status = "STOPPED";
    else                             status = "INIT";

    // 后缀标记（HMI 据此显示"暂停中"/"急停中"，并实现"暂停中只能预选一次"）：
    //   形如 "READY|PAUSED" / "STOPPED|PAUSED|ESTOP" / "FAULT|ESTOP"。HMI 用 contains 判断，不依赖顺序。
    //   同话题其它文本（"执行命令: xxx"、点动速度提示）都不含 '|'，HMI 解析时不会误判。
    //   注 1：ESTOP 对外是"急停中"的统一语义，来源有两个且内部严格分离：
    //         - 虚拟急停软件锁存 g_virtual_estop_active（HMI 可 toggle 解除）
    //         - 硬件急停电平 g_hw_estop_active（DI 驱动，松开即恢复，不可软件解除）
    //         HMI 现有的虚拟急停按钮由该标记点亮，物理急停期间点击会被 handle_virtual_io 互锁忽略，
    //         因此不会出现"界面说已解除、实物仍急停"的错位。
    //   注 2：EHW 表示"硬件急停仍处于按下状态"，仅供日志/Python 等非 HMI 消费方区分来源；
    //         命名刻意不含 "ESTOP" 子串 —— HMI 用 contains 匹配，写 ESTOP_HW 会误点亮虚拟急停按钮。
    if (g_paused.load()) status += "|PAUSED";
    if (is_virtual_estop_active() || g_hw_estop_active.load()) status += "|ESTOP";
    if (g_hw_estop_active.load()) status += "|EHW";

    auto msg = std_msgs::msg::String();
    msg.data = status;
    system_status_pub_->publish(msg);
}

void EthercatNode::handle_py_control_command(const std_msgs::msg::String::SharedPtr msg)
{
    std::string command = msg->data;

#if CONTROL_SOURCE_IO
    // IO控制模式下，禁止通过话题切换手动/自动模式（避免与IO控制冲突）
    if (command == CMD_START_MANUAL || command == CMD_START_AUTO)
    {
        // === 即使拒绝命令，也要检查模式冲突并上报故障 ===
        if (command == CMD_START_AUTO && fault_manager_)
        {
            bool any_axis_in_manual = false;
            for (auto &axis : servo_axes_)
            {
                if (axis->get_operation_mode() == OperationMode::MANUAL)
                {
                    any_axis_in_manual = true;
                    break;
                }
            }
            if (any_axis_in_manual)
            {
                // 手动模式下收到自动指令，上报故障 0x9004
                uint16_t fault_code = fault_manager_->handle_auto_command_in_manual_mode();
                std::stringstream warn_ss;
                warn_ss << "IO控制模式下，手动模式收到话题自动指令，已上报故障码: 0x"
                        << std::hex << std::setw(4) << std::setfill('0') << fault_code;
                print_warning(warn_ss.str());
            }
        }

        RCLCPP_WARN(this->get_logger(), "IO控制模式已启用，忽略话题模式切换命令: %s", command.c_str());
        return;
    }
#endif

    RCLCPP_INFO(this->get_logger(), "收到Python控制命令: %s", command.c_str());
    handle_control_command(command);
}

// 在轴初始化后调用的方法
void EthercatNode::initialize_after_axes() {
    RCLCPP_INFO(this->get_logger(), "开始初始化业务逻辑模块（延迟初始化）");

    // 检查轴是否已初始化
    if (servo_axes_.empty())
    {
        print_error("伺服轴未初始化，无法初始化业务逻辑模块");
        return;
    }

    // 业务逻辑模块已移至Python层
    RCLCPP_INFO(this->get_logger(), "业务逻辑模块初始化完成（框架模式）");
}

void EthercatNode::init_axes(ec_master_t *master)
{
    RCLCPP_INFO(this->get_logger(), "开始初始化伺服轴");

    // 清空现有轴对象，确保重新启动时初值正确
    // 方案A：IO轮询由主线程执行，不再停止IO监控（保持Modbus连接跨重启可用）
    servo_axes_.clear();
    last_target_positions_.clear(); // 同时清空位置记录

    // 添加从站1：第二个雷赛双轴驱动器
    /* auto axis1_1 = ServoAxisFactory::create_servo_axis(
        DriveBrand::LEISAI, "axis1_1", 0, AxisType::AXIS1, LEISAI_PRODUCT_CODE_3);  // 从站位置=1
    auto axis1_2 = ServoAxisFactory::create_servo_axis(
        DriveBrand::LEISAI, "axis1_2", 0, AxisType::AXIS2, LEISAI_PRODUCT_CODE_3);  // 从站位置=1（同一个从站）

    servo_axes_.push_back(std::move(axis1_1));
    servo_axes_.push_back(std::move(axis1_2));
    // 创建雷赛双轴驱动器 - 从站2
    auto axis2_1 = ServoAxisFactory::create_servo_axis(
        DriveBrand::LEISAI, "axis2_1", 1, AxisType::AXIS1, LEISAI_PRODUCT_CODE_1);
    auto axis2_2 = ServoAxisFactory::create_servo_axis(
        DriveBrand::LEISAI, "axis2_2", 1, AxisType::AXIS2, LEISAI_PRODUCT_CODE_1);

    servo_axes_.push_back(std::move(axis2_1));
    servo_axes_.push_back(std::move(axis2_2));

    servo_axes_.push_back(ServoAxisFactory::create_servo_axis(
        DriveBrand::LEISAI, "axis3", 2, AxisType::AXIS1, LEISAI_PRODUCT_CODE_2, 1.818)); // 轴，减速比2.0*20/22 =1.818,调式结果是1.8
    servo_axes_.push_back(ServoAxisFactory::create_servo_axis(
        DriveBrand::HUICHUAN, "axis4", 3, AxisType::AXIS1, 0, 7.34)); // 汇川轴，减速比9.0*28/34 =7.411,调式结果是7.5
    servo_axes_.push_back(ServoAxisFactory::create_servo_axis(
        DriveBrand::HUICHUAN, "axis5", 4, AxisType::AXIS1)); */

    // 从YAML配置文件加载轴配置
    std::string yaml_path = this->get_parameter("axes_config_path").as_string();
    if (!load_axes_from_yaml(yaml_path)) {
        RCLCPP_ERROR(this->get_logger(), "从YAML加载轴配置失败，使用空轴列表");
    }
    // 配置每个轴
    for (auto &axis : servo_axes_)
    {
        axis->configure(master);
    }
    // 所有轴配置完成后统一清除复位标志（回原轴已在各自 configure() 中读取该标志）
    if (g_reset_button_pressed.load())
    {
        g_reset_button_pressed.store(false);
    }

    // 打印每个轴的点动速度、自动速度和每转脉冲数
    for (auto &axis : servo_axes_)
    {
        RCLCPP_INFO(this->get_logger(), "轴 %s 点动速度: %.1f, 自动速度: %.1f, 每转脉冲数: %d",
                     axis->get_name().c_str(), axis->get_jog_speed(), axis->get_auto_speed(), axis->get_pulses_per_rev());
    }
    last_target_positions_.resize(servo_axes_.size(), 0.0);
    start_io_monitoring(); // 在轴初始化后启动IO监控，确保轴配置完成后才开始监控IO状态 todo3.11不能删，删了后暂停会出问题
    RCLCPP_INFO(this->get_logger(), "伺服轴初始化完成，共 %zu 个轴", servo_axes_.size());
}

bool EthercatNode::load_axes_from_yaml(const std::string &yaml_path)
{
    RCLCPP_INFO(this->get_logger(), "从YAML文件加载轴配置: %s", yaml_path.c_str());

    YAML::Node config;
    try {
        config = YAML::LoadFile(yaml_path);
    } catch (const YAML::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "无法加载YAML文件: %s, 错误: %s", yaml_path.c_str(), e.what());
        return false;
    }

    if (!config["servo_axes"]) {
        RCLCPP_ERROR(this->get_logger(), "YAML文件中未找到 'servo_axes' 节点");
        return false;
    }

    const YAML::Node &axes_node = config["servo_axes"];
    if (!axes_node.IsSequence()) {
        RCLCPP_ERROR(this->get_logger(), "'servo_axes' 必须是数组类型");
        return false;
    }

    for (size_t i = 0; i < axes_node.size(); ++i) {
        const YAML::Node &axis_cfg = axes_node[i];
        try {
            std::string brand_str = axis_cfg["brand"].as<std::string>();
            std::string name = axis_cfg["name"].as<std::string>();
            uint16_t position = axis_cfg["position"].as<uint16_t>();
            std::string axis_type_str = axis_cfg["axis_type"].as<std::string>();
            uint32_t product_code = axis_cfg["product_code"] ? axis_cfg["product_code"].as<uint32_t>() : 0;
            double gear_ratio = axis_cfg["gear_ratio"] ? axis_cfg["gear_ratio"].as<double>() : 1.0;
            std::string calc_mode_str = axis_cfg["calc_mode"] ? axis_cfg["calc_mode"].as<std::string>() : "DISPLACEMENT";
            // operation_mode: "homing" 表示该轴为回原轴（复位流程中切换为 HM 模式）
            std::string operation_mode_str = axis_cfg["operation_mode"] ? axis_cfg["operation_mode"].as<std::string>() : "csp";

            DriveBrand brand = ServoAxisFactory::string_to_brand(brand_str);
            AxisType axis_type = ServoAxisFactory::string_to_axis_type(axis_type_str);
            AxisCalcMode calc_mode = ServoAxisFactory::string_to_calc_mode(calc_mode_str);

            auto axis = ServoAxisFactory::create_servo_axis(
                brand, name, position, axis_type, product_code, gear_ratio, calc_mode);

            // 标记回原轴（复位流程中切换为 HM 回原模式），替代原先硬编码 "axis3" 判断
            if (operation_mode_str == "homing" || operation_mode_str == "HOMING" ||
                operation_mode_str == "hm" || operation_mode_str == "HM") {
                axis->set_homing_mode(true);
                RCLCPP_INFO(this->get_logger(), "轴 %s 标记为回原轴 (operation_mode=homing)", name.c_str());
            }

            // 从YAML读取速度范围（角度轴: °/s, 位移轴: mm/s）
            // 必须先设置范围，再设置点动速度，否则 set_jog_speed 会因默认范围校验失败
            if (axis_cfg["min_jog_speed"]) {
                double min_speed = axis_cfg["min_jog_speed"].as<double>();
                axis->set_min_jog_speed(min_speed);
            }
            if (axis_cfg["max_jog_speed"]) {
                double max_speed = axis_cfg["max_jog_speed"].as<double>();
                axis->set_max_jog_speed(max_speed);
            }

            // 从YAML读取点动速度并设置（角度轴: °/s, 位移轴: mm/s）
            if (axis_cfg["jog_speed"]) {
                double jog_speed = axis_cfg["jog_speed"].as<double>();
                axis->set_jog_speed(jog_speed);
                RCLCPP_INFO(this->get_logger(), "轴 %s 点动速度设置为: %.1f",
                            name.c_str(), jog_speed);
            }

            // 从YAML读取自动速度并设置（角度轴: °/s, 位移轴: mm/s）
            if (axis_cfg["auto_speed"]) {
                double auto_speed = axis_cfg["auto_speed"].as<double>();
                if (axis->set_auto_speed(auto_speed)) {
                    RCLCPP_INFO(this->get_logger(), "轴 %s 自动速度设置为: %.1f",
                                name.c_str(), auto_speed);
                } else {
                    // 非法配置必须显式上报，避免静默回落到默认值后现场"速度不对却查不到原因"
                    RCLCPP_ERROR(this->get_logger(),
                        "轴 %s 的 auto_speed=%.3f 非法（须在 (0, %.1f] 且每周期步长>=1），"
                        "已保留默认值 %.1f",
                        name.c_str(), auto_speed, axis->get_max_jog_speed(),
                        axis->get_auto_speed());
                }
            }

            // 从YAML读取自动模式梯形速度曲线的加减速参数
            // 默认 4 × max_jog_speed：任何速度百分比下都能在 ~0.25s 内达到巡航速度
            {
                const double max_jog = axis->get_max_jog_speed();
                const double accel = axis_cfg["accel"] ? axis_cfg["accel"].as<double>()
                                                       : 4.0 * max_jog;
                const double decel = axis_cfg["decel"] ? axis_cfg["decel"].as<double>()
                                                       : accel;
                if (axis->set_auto_profile(accel, decel)) {
                    RCLCPP_INFO(this->get_logger(),
                        "轴 %s 梯形曲线: accel=%.1f, decel=%.1f (%s/s²), 达到最大速度约需 %.2fs",
                        name.c_str(), accel, decel,
                        calc_mode == AxisCalcMode::ANGLE ? "°" : "mm",
                        (accel > 0.0) ? (max_jog / accel) : 0.0);
                } else {
                    RCLCPP_ERROR(this->get_logger(),
                        "轴 %s 的 accel=%.3f 非法（必须为正），已忽略；"
                        "该轴自动模式将退回限速一步式（无加减速）",
                        name.c_str(), accel);
                }
            }

            // 从YAML读取手动点动软起动参数（软起动 / 软停止 / 软换向）
            // 缺省（不配或填 0）：jog_accel = 4 × jog_speed      → 0→点动速度约 0.25s
            //                    jog_decel = 20 × max_jog_speed  → 全速刹停 2.5~15ms，滑行小
            if (axis_cfg["jog_accel"] || axis_cfg["jog_decel"]) {
                const double jog_accel = axis_cfg["jog_accel"] ? axis_cfg["jog_accel"].as<double>() : 0.0;
                const double jog_decel = axis_cfg["jog_decel"] ? axis_cfg["jog_decel"].as<double>() : 0.0;
                if (axis->set_jog_profile(jog_accel, jog_decel)) {
                    RCLCPP_INFO(this->get_logger(),
                        "轴 %s 手动点动软起动: jog_accel=%.1f, jog_decel=%.1f (%s/s²)，0 表示采用缺省",
                        name.c_str(), jog_accel, jog_decel,
                        calc_mode == AxisCalcMode::ANGLE ? "°" : "mm");
                } else {
                    RCLCPP_ERROR(this->get_logger(),
                        "轴 %s 的 jog_accel=%.3f / jog_decel=%.3f 非法（必须为正），已忽略并沿用缺省",
                        name.c_str(), jog_accel, jog_decel);
                }
            }

            // 从YAML读取角度软限位（仅角度模式轴有效，单位：度）
            // min_angle: 最小允许角度（负数表示反向限位）
            // max_angle: 最大允许角度（正数表示正向限位）
            if (axis_cfg["min_angle"] || axis_cfg["max_angle"]) {
                double min_angle = axis_cfg["min_angle"] ? axis_cfg["min_angle"].as<double>() : -180.0;
                double max_angle = axis_cfg["max_angle"] ? axis_cfg["max_angle"].as<double>() : 180.0;
                axis->set_angle_limits(min_angle, max_angle);
                RCLCPP_INFO(this->get_logger(), "轴 %s 角度软限位: [%.1f°, %.1f°]",
                            name.c_str(), min_angle, max_angle);
            }

            servo_axes_.push_back(std::move(axis));

            RCLCPP_INFO(this->get_logger(), "已加载轴[%zu]: brand=%s, name=%s, position=%d, axis_type=%s, product_code=%u, gear_ratio=%.3f, calc_mode=%s",
                        i, brand_str.c_str(), name.c_str(), position, axis_type_str.c_str(),
                        product_code, gear_ratio, calc_mode_str.c_str());
        } catch (const YAML::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "解析轴[%zu]配置失败: %s", i, e.what());
            return false;
        } catch (const std::invalid_argument &e) {
            RCLCPP_ERROR(this->get_logger(), "轴[%zu]参数无效: %s", i, e.what());
            return false;
        }
    }

    if (servo_axes_.empty()) {
        RCLCPP_WARN(this->get_logger(), "YAML配置中没有定义任何轴");
        return false;
    }

    return true;
}

void EthercatNode::register_pdo_entries(ec_domain_t *domain1)
{
    RCLCPP_INFO(this->get_logger(), "开始注册PDO条目");

    const int entries_per_axis = 5;
    int total_entries = servo_axes_.size() * entries_per_axis + 1;

    ec_pdo_entry_reg_t *reg_list = new ec_pdo_entry_reg_t[total_entries];
    int index = 0;

    // 每个轴注册自己的PDO条目
    for (auto &axis : servo_axes_)
    {
        axis->register_pdo_entries(reg_list, index);
    }

    reg_list[index] = {};

    if (ecrt_domain_reg_pdo_entry_list(domain1, reg_list))
    {
        RCLCPP_FATAL(this->get_logger(), "PDO注册失败");
    }

    delete[] reg_list;
    RCLCPP_INFO(this->get_logger(), "PDO条目注册完成");
}

void EthercatNode::handle_axes_state_machines(uint8_t *domain1_pd)
{
    for (auto &axis : servo_axes_)
    {
        axis->handle_state_machine(domain1_pd);
    }
}

std::vector<std::shared_ptr<ServoAxisBase>> &EthercatNode::get_servo_axes()
{
    return servo_axes_;
}

std::vector<ec_slave_config_t *> EthercatNode::get_all_slave_configs()
{
    std::vector<ec_slave_config_t *> configs;
    for (auto &axis : servo_axes_)
    {
        configs.push_back(axis->get_slave_config());
    }
    return configs;
}

void EthercatNode::publish_joint_states()
{
    // 添加关闭检查
    if (node_shutting_down_.load() || !rclcpp::ok())
    {
        return;
    }

    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";

    // 收集所有轴的名称
    for (auto &axis : servo_axes_)
    {
        msg.name.push_back(axis->get_name());
    }

    // 使用绝对编码器原始脉冲直读，不受模式切换影响
    for (auto &axis : servo_axes_)
    {
        int32_t encoder_pulses = axis->get_raw_encoder_pulses();

        if (axis->get_calc_mode() == AxisCalcMode::ANGLE) {
            // ========== 角度模式 (rad) ==========
            // 绝对编码器脉冲直接换算（已含减速比、每转脉冲数）
            double deg = axis->pulses_to_angle(encoder_pulses);
            double rad = deg * M_PI / 180.0;
            msg.position.push_back(rad);
        } else {
            // ========== 位移模式 (mm) ==========
            // 绝对编码器脉冲直接换算（已含减速比、导程、每转脉冲数）
            double displacement_mm = axis->pulses_to_displacement(encoder_pulses);
            msg.position.push_back(displacement_mm);
        }
    }

    joint_state_pub_->publish(msg);

    static int log_counter = 0;
    if (log_counter++ >= 100)
    {
        log_counter = 0;
        RCLCPP_DEBUG(this->get_logger(), "发布关节状态: %zu个关节", msg.name.size());
    }
}

void EthercatNode::handle_control_command(const std::string &command)
{
    if (node_shutting_down_.load() || !rclcpp::ok())
    {
        return;
    }

    // === 命令去重防抖：相同命令0.5秒内不重复执行 ===
    auto now = this->now();
    if (command == last_command_ && (now - last_command_time_).seconds() < CMD_DEDUP_SEC)
    {
        RCLCPP_DEBUG(this->get_logger(), "忽略重复命令: %s (%.1fs内)", command.c_str(), CMD_DEDUP_SEC);
        return;
    }
    last_command_ = command;
    last_command_time_ = now;

    RCLCPP_INFO(this->get_logger(), "收到控制命令: %s", command.c_str());

    // 发布系统状态
    auto status_msg = std_msgs::msg::String();
    status_msg.data = "执行命令: " + command;
    system_status_pub_->publish(status_msg);

    if (command == CMD_START_MANUAL)
    {
        // 暂停中只"预选"，不立即切模式：
        //   若立即切，轴会进 MANUAL 模式 → is_running() 为真 → 位移指令照常执行，暂停被打穿；
        //   而且解除暂停时又会按 DI 重选，操作员的选择被覆盖。
        if (g_paused.load())
        {
            g_pending_mode.store(1, std::memory_order_relaxed);
            RCLCPP_INFO(this->get_logger(), "暂停中，已预选手动模式（解除暂停时生效）");
        }
        else
        {
            // 手动模式
            for (auto &axis : servo_axes_)
            {
                axis->start_manual_mode();
            }
            RCLCPP_INFO(this->get_logger(), "所有轴接收到手动模式启动命令");
        }
    }
    else if (command == CMD_START_AUTO)
    {
        if (g_paused.load())
        {
            g_pending_mode.store(2, std::memory_order_relaxed);
            RCLCPP_INFO(this->get_logger(), "暂停中，已预选自动模式（解除暂停时生效）");
        }
        else
        {
            // 自动模式
            for (auto &axis : servo_axes_)
            {
                axis->start_auto_mode();
            }
            RCLCPP_INFO(this->get_logger(), "所有轴接收到自动模式启动命令");
        }
    }
    else if (command == CMD_STOP)
    {
        // 停止模式
        for (auto &axis : servo_axes_)
        {
            axis->stop();
        }
        // 防御性清理：停机后若再进暂停/恢复，不该沿用旧预选
        g_pending_mode.store(0, std::memory_order_relaxed);
        RCLCPP_INFO(this->get_logger(), "所有轴接收到停止命令");
        // 清除所有系统故障和告警
        if (fault_manager_)
        {
            fault_manager_->clear_all_faults();
            RCLCPP_INFO(this->get_logger(), "已清除所有系统故障和告警");
        }
    }
    else if (command == CMD_CLEAR_FAULT)
    {
        // 清除故障（清故障是"故障后恢复"的必经步骤，此处清预选 = 等效"故障即清"）
        for (auto &axis : servo_axes_)
        {
            axis->clear_fault();
        }
        g_pending_mode.store(0, std::memory_order_relaxed);
        RCLCPP_INFO(this->get_logger(), "所有轴接收到清除故障命令");
    }
    else if (command == CMD_RESET)
    {
        // 重置（重新来过，清预选）
        for (auto &axis : servo_axes_)
        {
            axis->reset_axis();
        }
        g_pending_mode.store(0, std::memory_order_relaxed);
        RCLCPP_INFO(this->get_logger(), "所有轴接收到重置命令");
    }
    else if (command == "clear_all_faults")
    {
        // 清除所有系统故障和告警（Python 通道走这条，同样清预选）
        g_pending_mode.store(0, std::memory_order_relaxed);
        if (fault_manager_)
        {
            fault_manager_->clear_all_faults();
            RCLCPP_INFO(this->get_logger(), "已清除所有系统故障和告警");
        }
    }
    else
    {
        print_warning("未知控制命令: " + command);
    }
}

// 实现新的位移指令处理函数
void EthercatNode::handle_displacement_command(const std_msgs::msg::String::SharedPtr msg)
{
    if (servo_axes_.empty())
    {
        RCLCPP_DEBUG(this->get_logger(), "等待伺服轴初始化...");
        return;
    }

    std::string command = msg->data;
    RCLCPP_INFO(this->get_logger(), "收到位移指令: %s", command.c_str());

    // 解析命令
    std::vector<std::pair<std::string, double>> axis_commands;
    if (!parse_displacement_command(command, axis_commands))
    {
        print_error("位移指令解析失败: " + command);
        return;
    }

    // 处理每个轴的位移命令
    for (const auto &axis_cmd : axis_commands)
    {
        const std::string &axis_name = axis_cmd.first;
        double displacement = axis_cmd.second;

        printf("解析位移指令: %s = %.6fmm\n", axis_name.c_str(), displacement);

        // 查找对应的轴
        bool axis_found = false;
        for (size_t i = 0; i < servo_axes_.size(); ++i)
        {
            if (servo_axes_[i]->get_name() == axis_name)
            {
                // 单轴控制
                if (servo_axes_[i]->is_running())
                {
                    handle_axis_command(i, displacement);
                }
                axis_found = true;
                break;
            }
        }

        if (!axis_found)
        {
            print_error("未找到轴: " + axis_name);
        }
    }
}

// 实现位移指令解析函数
bool EthercatNode::parse_displacement_command(const std::string &command,
                                              std::vector<std::pair<std::string, double>> &axis_commands)
{
    axis_commands.clear();

    // 支持多种格式：
    // 1. 单个轴: "axis4:1.0"
    // 2. 多个轴: "axis4:1.0;axis5:2.0"

    // 解析新格式
    std::vector<std::string> parts;
    size_t start = 0;
    size_t end = command.find(';');

    while (end != std::string::npos)
    {
        parts.push_back(command.substr(start, end - start));
        start = end + 1;
        end = command.find(';', start);
    }
    parts.push_back(command.substr(start));

    for (const auto &part : parts)
    {
        size_t colon_pos = part.find(':');
        if (colon_pos == std::string::npos)
        {
            print_error("无效指令格式，缺少冒号: " + part);
            return false;
        }

        std::string axis_name = part.substr(0, colon_pos);
        std::string value_str = part.substr(colon_pos + 1);

        // 去除空格
        axis_name.erase(0, axis_name.find_first_not_of(" \t"));
        axis_name.erase(axis_name.find_last_not_of(" \t") + 1);
        value_str.erase(0, value_str.find_first_not_of(" \t"));
        value_str.erase(value_str.find_last_not_of(" \t") + 1);

        if (axis_name.empty() || value_str.empty())
        {
            print_error("轴名或值为空: " + part);
            return false;
        }

        try
        {
            double value = std::stod(value_str);
            axis_commands.push_back({axis_name, value});
        }
        catch (const std::exception &e)
        {
            print_error("数值转换失败: " + value_str + ", 错误: " + e.what());
            return false;
        }
    }

    return true;
}

// 使用轴配置的默认自动速度（auto_speed_）
void EthercatNode::handle_axis_command(size_t axis_index, double newTargetPosition)
{
    if (axis_index < servo_axes_.size())
    {
        // 明确表态：本次使用默认 auto_speed_，清除上一次可能残留的百分比速度
        servo_axes_[axis_index]->clear_move_speed();
        servo_axes_[axis_index]->set_target_displacement(newTargetPosition);
        servo_axes_[axis_index]->set_displacement_updated(true);

        RCLCPP_DEBUG(this->get_logger(), "轴[%zu] 位移指令: %.4fmm",
                     axis_index, newTargetPosition);
    }
}

// 使用本次指定的移动速度（由 /xxx_command_pct 的百分比折算而来）
void EthercatNode::handle_axis_command(size_t axis_index, double newTargetPosition, double speed)
{
    if (axis_index >= servo_axes_.size())
    {
        return;
    }

    auto &axis = servo_axes_[axis_index];
    // 必须先设速度、再下发目标：否则实时线程可能"新目标配旧速度"跑一段
    // set_move_speed 内部已校验 0 < speed <= max_jog_speed_ 且每周期步长 >= 1
    if (!axis->set_move_speed(speed))
    {
        print_warning("轴 " + axis->get_name() + " 速度不可实现，指令被拒绝");
        return;   // 拒绝，且不下发目标
    }
    axis->set_target_displacement(newTargetPosition);
    axis->set_displacement_updated(true);

    RCLCPP_DEBUG(this->get_logger(), "轴[%zu] 位移指令: %.4fmm, 速度 %.3f",
                 axis_index, newTargetPosition, speed);
}

// ========== 角度控制功能 ==========
// 弧度转毫米: rad -> 转数 -> mm
double EthercatNode::rad_to_mm(double rad)
{
    const double SCREW_LEAD = 1.0;      // 丝杠导程 10mm/rev
    const double GEAR_RATIO = 1.0;       // 减速比
    // 转数 = 弧度 / (2π)
    // mm = 转数 * 减速比 * 丝杠导程
    double revolutions = rad / (2 * M_PI);
    return revolutions * GEAR_RATIO * SCREW_LEAD;
}

// 处理角度控制命令 (格式与位移命令相同: "axis4:10.0")
void EthercatNode::handle_angle_command(const std_msgs::msg::String::SharedPtr msg)
{
    if (servo_axes_.empty())
    {
        RCLCPP_DEBUG(this->get_logger(), "等待伺服轴初始化...");
        return;
    }

    std::string command = msg->data;
    RCLCPP_INFO(this->get_logger(), "收到角度命令: %s", command.c_str());

    // 使用与位移命令相同的解析方法
    std::vector<std::pair<std::string, double>> axis_commands;
    if (!parse_displacement_command(command, axis_commands))
    {
        print_error("无法解析角度命令: " + command);
        return;
    }

    // 处理每个轴的角度指令 (Python发送rad, C++内部angle_to_pulses使用度)
    for (const auto &cmd : axis_commands)
    {
        const std::string &axis_name = cmd.first;
        double angle_rad = cmd.second;

        // 弧度转度：Python发送角度指令单位为rad，但 servo 侧 angle_to_pulses 期望单位为度
        double angle_deg = angle_rad * 180.0 / M_PI;

        // 查找轴索引
        size_t axis_index = 0;
        bool found = false;
        for (size_t i = 0; i < servo_axes_.size(); i++)
        {
            if (servo_axes_[i]->get_name() == axis_name)
            {
                axis_index = i;
                found = true;
                break;
            }
        }

        if (!found)
        {
            print_error("未找到轴: " + axis_name);
            continue;
        }

        // 角度软限位检查与钳制（仅已启用限位的轴生效）
        double clamped_deg = servo_axes_[axis_index]->clamp_angle(angle_deg);
        if (clamped_deg != angle_deg && servo_axes_[axis_index]->has_angle_limits()) {
            RCLCPP_WARN(this->get_logger(),
                "轴 %s 角度指令 %.2f° 超出软限位 [%.1f°, %.1f°]，已钳制为 %.2f°",
                axis_name.c_str(), angle_deg,
                servo_axes_[axis_index]->get_min_angle(),
                servo_axes_[axis_index]->get_max_angle(),
                clamped_deg);
        }
        // 发送角度指令（度），轴侧 angle_to_pulses 将度转换为脉冲
        handle_axis_command(axis_index, clamped_deg);
        RCLCPP_INFO(this->get_logger(), "轴 %s 角度指令: %.4f rad (%.2f°)",
                    axis_name.c_str(), angle_rad, clamped_deg);
    }
}

// ========== 速度百分比指令（_pct 话题）==========
// 通用解析："轴名:值" 或 "轴名:值:整数百分比"，多轴用 ';' 分隔
// 与 parse_displacement_command 的关键区别：这里自行按 ':' 切分并严格校验，
// 绝不把整段交给 std::stod —— 否则 "100.0:70" 会被静默解析成 100.0，百分比被无声吞掉
bool EthercatNode::parse_command_ex(const std::string &command,
                                    std::vector<AxisCmdEx> &axis_commands,
                                    bool need_percent)
{
    axis_commands.clear();

    std::vector<std::string> parts;
    size_t start = 0;
    size_t end = command.find(';');
    while (end != std::string::npos)
    {
        parts.push_back(command.substr(start, end - start));
        start = end + 1;
        end = command.find(';', start);
    }
    parts.push_back(command.substr(start));

    for (const auto &part : parts)
    {
        // 按 ':' 切分（轴名不含冒号）
        std::vector<std::string> tokens;
        size_t t_start = 0;
        size_t t_end = part.find(':');
        while (t_end != std::string::npos)
        {
            tokens.push_back(part.substr(t_start, t_end - t_start));
            t_start = t_end + 1;
            t_end = part.find(':', t_start);
        }
        tokens.push_back(part.substr(t_start));

        // 去除空格
        for (auto &t : tokens)
        {
            t.erase(0, t.find_first_not_of(" \t"));
            if (!t.empty())
            {
                t.erase(t.find_last_not_of(" \t") + 1);
            }
        }

        const size_t expect_tokens = need_percent ? 3u : 2u;
        if (tokens.size() != expect_tokens)
        {
            print_error("指令段格式错误（应为 轴名:值" +
                        std::string(need_percent ? ":百分比" : "") + "）: " + part);
            return false;
        }
        if (tokens[0].empty() || tokens[1].empty())
        {
            print_error("轴名或值为空: " + part);
            return false;
        }

        double value = 0.0;
        try
        {
            size_t pos = 0;
            value = std::stod(tokens[1], &pos);
            if (pos != tokens[1].size())   // 拒绝 "100abc" 这类残留
            {
                print_error("目标值格式非法: " + tokens[1]);
                return false;
            }
        }
        catch (const std::exception &e)
        {
            print_error("数值转换失败: " + tokens[1] + ", 错误: " + e.what());
            return false;
        }

        int32_t percent = 0;
        if (need_percent)
        {
            if (tokens[2].empty())
            {
                print_error("百分比为空: " + part);
                return false;
            }
            try
            {
                size_t pos = 0;
                percent = std::stoi(tokens[2], &pos);
                // 强制整数：拒绝 "70.0" / "70abc" 等写法
                if (pos != tokens[2].size())
                {
                    print_error("百分比必须为纯整数（如 70）: " + tokens[2]);
                    return false;
                }
            }
            catch (const std::exception &e)
            {
                print_error("百分比转换失败: " + tokens[2] + ", 错误: " + e.what());
                return false;
            }
            if (percent < 5 || percent > 100)
            {
                print_error("百分比超出范围 [5, 100]: " + tokens[2]);
                return false;
            }
        }

        axis_commands.push_back({tokens[0], value, percent});
    }

    return true;
}

// 位移轴速度百分比指令：/displacement_command_pct  "axis4:100.0:70"
void EthercatNode::handle_displacement_pct_command(const std_msgs::msg::String::SharedPtr msg)
{
    if (servo_axes_.empty())
    {
        RCLCPP_DEBUG(this->get_logger(), "等待伺服轴初始化...");
        return;
    }

    std::string command = msg->data;
    RCLCPP_INFO(this->get_logger(), "收到位移百分比指令: %s", command.c_str());

    std::vector<AxisCmdEx> axis_commands;
    if (!parse_command_ex(command, axis_commands, true))
    {
        print_error("位移百分比指令解析失败: " + command);
        return;
    }

    for (const auto &cmd : axis_commands)
    {
        bool axis_found = false;
        for (size_t i = 0; i < servo_axes_.size(); ++i)
        {
            if (servo_axes_[i]->get_name() != cmd.name)
            {
                continue;
            }
            axis_found = true;
            if (!servo_axes_[i]->is_running())
            {
                RCLCPP_WARN(this->get_logger(),
                    "轴 %s 当前不在手动/自动模式，忽略指令", cmd.name.c_str());
                break;
            }
            // 目标速度 = 百分比 × 该轴最大速度(max_jog_speed_)
            const double speed = cmd.percent / 100.0 * servo_axes_[i]->get_max_jog_speed();
            handle_axis_command(i, cmd.value, speed);
            RCLCPP_INFO(this->get_logger(),
                "轴 %s 位移指令: %.3f mm, %d%% -> %.3f mm/s",
                cmd.name.c_str(), cmd.value, cmd.percent, speed);
            break;
        }
        if (!axis_found)
        {
            print_error("未找到轴: " + cmd.name);
        }
    }
}

// 角度轴速度百分比指令：/angle_command_pct  "axis6:1.5708:70"
void EthercatNode::handle_angle_pct_command(const std_msgs::msg::String::SharedPtr msg)
{
    if (servo_axes_.empty())
    {
        RCLCPP_DEBUG(this->get_logger(), "等待伺服轴初始化...");
        return;
    }

    std::string command = msg->data;
    RCLCPP_INFO(this->get_logger(), "收到角度百分比指令: %s", command.c_str());

    std::vector<AxisCmdEx> axis_commands;
    if (!parse_command_ex(command, axis_commands, true))
    {
        print_error("角度百分比指令解析失败: " + command);
        return;
    }

    for (const auto &cmd : axis_commands)
    {
        // 上层发送 rad，C++ 内部 angle_to_pulses 使用度
        const double angle_deg = cmd.value * 180.0 / M_PI;

        size_t axis_index = 0;
        bool found = false;
        for (size_t i = 0; i < servo_axes_.size(); i++)
        {
            if (servo_axes_[i]->get_name() == cmd.name)
            {
                axis_index = i;
                found = true;
                break;
            }
        }
        if (!found)
        {
            print_error("未找到轴: " + cmd.name);
            continue;
        }
        if (!servo_axes_[axis_index]->is_running())
        {
            RCLCPP_WARN(this->get_logger(),
                "轴 %s 当前不在手动/自动模式，忽略指令", cmd.name.c_str());
            continue;
        }

        // 角度软限位检查与钳制（仅已启用限位的轴生效；只影响目标，不影响速度）
        const double clamped_deg = servo_axes_[axis_index]->clamp_angle(angle_deg);
        if (clamped_deg != angle_deg && servo_axes_[axis_index]->has_angle_limits())
        {
            RCLCPP_WARN(this->get_logger(),
                "轴 %s 角度指令 %.2f° 超出软限位 [%.1f°, %.1f°]，已钳制为 %.2f°",
                cmd.name.c_str(), angle_deg,
                servo_axes_[axis_index]->get_min_angle(),
                servo_axes_[axis_index]->get_max_angle(),
                clamped_deg);
        }

        // 目标速度 = 百分比 × 该轴最大速度(max_jog_speed_)，角度轴单位 °/s
        const double speed = cmd.percent / 100.0 * servo_axes_[axis_index]->get_max_jog_speed();
        handle_axis_command(axis_index, clamped_deg, speed);
        RCLCPP_INFO(this->get_logger(),
            "轴 %s 角度指令: %.4f rad (%.2f°), %d%% -> %.3f °/s",
            cmd.name.c_str(), cmd.value, clamped_deg, cmd.percent, speed);
    }
}

double EthercatNode::pulses_to_displacement(int32_t pulses, int32_t initial_pulses)
{
    const double SCREW_LEAD = 0.314;
    const double GEAR_RATIO = 1.0;
    // 注意：此函数为全局通用计算，若需要按轴区分请使用 ServoAxisBase::pulses_to_displacement()
    const int PULSES_PER_REV = 36000;
    return static_cast<double>((pulses - initial_pulses) * SCREW_LEAD / (GEAR_RATIO * PULSES_PER_REV));
}

void EthercatNode::handle_jog_command(const std_msgs::msg::String::SharedPtr msg)
{
    std::string command = msg->data;
    RCLCPP_INFO(this->get_logger(), "收到点动命令: %s", command.c_str());

    // 暂停中（g_paused）直接忽略点动指令。
    // /jog_command 没有模式检查，而 READY 下点动标志只置不清（会残留到解除暂停后执行），
    // 这里做闸门，配合 main 在翻转暂停时清标志，彻底消除"恢复瞬间突然动一下"。
    if (g_paused.load())
    {
        RCLCPP_INFO(this->get_logger(), "暂停中，忽略点动命令: %s", command.c_str());
        return;
    }
    
    // 解析格式："axis1_1:forward" 或 "axis1_1:reverse" 或 "axis1_1:stop"
    size_t colon_pos = command.find(':');
    if (colon_pos == std::string::npos)
    {
        print_error("无效命令格式，应为 '轴名:命令'");
        return;
    }

    std::string axis_name = command.substr(0, colon_pos);
    std::string jog_cmd = command.substr(colon_pos + 1);

    // if (axis_name == "all") {
    //     // 控制所有轴
    //     for (auto& axis : servo_axes_) {
    //         execute_jog_command(axis, jog_cmd);
    //     }
    // } else {
    // 控制指定轴
    for (auto &axis : servo_axes_)
    {
        if (axis->get_name() == axis_name)
        {
            if (jog_cmd == "forward")
            {
                axis->jog_forward();
            }
            else if (jog_cmd == "reverse")
            {
                axis->jog_reverse();
            }
            else if (jog_cmd == "stop")
            {
                axis->jog_stop();
            }
            break;
        }
    }
    // }
}

// ========== IO模块相关函数 ==========

// IO 初始化失败的统一出口（fail fast）
void EthercatNode::fail_fast_io(const std::string &reason)
{
    const std::string msg = "IO 初始化失败，系统退出: " + reason;
    print_error(msg);
    RCLCPP_FATAL(this->get_logger(), "%s", msg.c_str());

    io_running_.store(false);
    g_should_exit.store(true);
    node_shutting_down_.store(true);
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
}

void EthercatNode::start_io_monitoring()
{
    if (io_running_.load())
    {
        RCLCPP_WARN(this->get_logger(), "IO监控线程已在运行");
        return;
    }

    // 从YAML加载IO配置
    // 路径由 io_config_path 参数给出；契约 C1~C4 任一被违反 → 拒绝启动（fail fast）
    std::string io_config_path = this->get_parameter("io_config_path").as_string();
    if (!load_io_config(io_config_path)) {
        fail_fast_io("IO 配置不合法（详见上方 [IO] 报错）");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "IO 配置已加载: %s (DI %d 点 / DO %d 点)",
                io_config_path.c_str(), g_di_count, g_do_count);

    // 初始化Modbus接口
    {
        int modbus_result = 0;
        if (g_io_integrated_mode) {
            const char *di_ip = (is_di_module_enabled() || is_do_module_enabled()) ? g_di_device_ip.c_str() : NULL;
            modbus_result = init_modbus_interface(di_ip, g_modbus_port, g_modbus_slave_id,
                                                  NULL, 0, 0);
        } else {
            const char *di_ip = is_di_module_enabled() ? g_di_device_ip.c_str() : NULL;
            const char *do_ip = is_do_module_enabled() ? g_do_device_ip.c_str() : NULL;
            modbus_result = init_modbus_interface(di_ip, g_modbus_port, g_modbus_slave_id,
                                                  do_ip, g_modbus_port, g_modbus_slave_id);
        }
        if (modbus_result != 0) {
            fail_fast_io("Modbus 初始化失败（设备不可达/端口错误，详见上方连接错误）");
            return;
        }
    }

    io_running_.store(true);

    // 方案A：不再创建IO监控线程
    // DI读取与信号处理统一由main.cpp主循环执行（约100ms周期）
    if (!is_di_module_enabled()) {
        print_warning("DI模块已禁用");
    }
    if (!is_do_module_enabled()) {
        print_warning("DO模块已禁用");
    }
    RCLCPP_INFO(this->get_logger(), "IO接口初始化完成（主线程轮询模式）");
}

void EthercatNode::stop_io_monitoring()
{
    if (!io_running_.load())
    {
        return;
    }

    io_running_.store(false);

    // 方案A：无IO监控线程需要等待，仅清理Modbus连接（完全关闭时调用）
    cleanup_modbus_interface();
    pthread_mutex_destroy(&io_mutex_);
    RCLCPP_INFO(this->get_logger(), "IO接口已停止并清理Modbus连接");
}

void EthercatNode::handle_io_signals(DI_Interface di)
{
    monitor_di_changes(di);
    pthread_mutex_lock(&io_mutex_);
    current_di_status_ = di;
    pthread_mutex_unlock(&io_mutex_);

    // 此处仅检测按钮状态并设置全局标志，不直接控制DO
    // 注：系统就绪灯语（黄闪 → 绿闪 → 绿常亮）的判定已统一收敛到 main.cpp 主循环，
    //     判定源为“实时 al_states==0x08 + 全部轴 is_ready()”；此处不再重复判定，
    //     避免全局 master_state（1Hz 刷新、暂停释放主站后仍残留 0x08）造成二次误触发。

    // 新增：IO控制模式切换（仅在宏开关启用时生效）
    // 注意：手自动按钮是"可选角色"（di_roles.manual_auto_button）：
    //       老机型在 io_config.yaml 配置了该角色 → 按 DI 电平（高=自动/低=手动）；
    //       圣白机型点表没有该按钮 → 角色未配置 → 按 di_module.default_operation_mode
    //       （auto|manual，缺省 manual）决定，从而支持"无按钮机型上电进自动"。
#if CONTROL_SOURCE_IO
    // 手自动来源（唯一判定处）：
    //   1) 配置了 di_roles.manual_auto_button → 以该 DI 电平为准（高=自动、低=手动，无极性反转）；
    //   2) 该可选角色未配置（如圣白机型无手自动按钮）→ 退回 di_module.default_operation_mode
    //      （auto|manual，缺省 manual）。
    // 注：这里是独立口径，不要改用 is_ready() 等其它判定（灯语判定在 main.cpp 是另一套）。
    const bool has_manual_auto_btn = (g_di_role_offset[DI_ROLE_MANUAL_AUTO_BUTTON] >= 0);
    const bool current_manual_auto_state =
        has_manual_auto_btn ? di.manual_auto_button : g_default_auto_mode;

    // 检查所有轴是否都处于READY状态
    bool all_axes_ready = true;
    for (auto &axis : servo_axes_)
    {
        AxisState state = axis->get_current_state();
        if (state != AxisState::READY)
        {
            all_axes_ready = false;
            break;
        }
    }

    // 暂停中（g_paused）不自动进模式：
    //   暂停时轴已被 stop() 带回 READY，若仍按"全轴 READY"进模式，暂停会被立刻解除，
    //   "再按一次暂停才恢复"就无从触发。
    //   同时把边沿记忆压成 false —— 解除暂停后即使 main 那次进模式命令失败，仍能重新进模式。
    if (g_paused.load())
    {
        last_all_axes_ready_ = false;
    }
    // 所有轴都处于READY状态时，尝试模式切换
    else if (all_axes_ready)
    {
        // 仅在状态变化时打印日志（边沿检测）
        if (!last_all_axes_ready_ || last_manual_auto_state_ != current_manual_auto_state)
        {
            RCLCPP_INFO(this->get_logger(), "所有轴已就绪，准备模式切换。手自动状态: %s (来源: %s)",
                        current_manual_auto_state ? "自动" : "手动",
                        has_manual_auto_btn ? "DI手自动按钮电平" : "配置默认模式");

            // 触发模式切换命令
            std::string command = current_manual_auto_state ? CMD_START_AUTO : CMD_START_MANUAL;
            handle_control_command(command); // 通过统一命令处理

            RCLCPP_INFO(this->get_logger(), "已发送%s模式切换命令",
                        current_manual_auto_state ? "自动" : "手动");
        }
        last_all_axes_ready_ = true;
    }
    else
    {
        last_all_axes_ready_ = false;

        // 记录哪些轴未就绪（用于调试）
        static int log_counter = 0;
        if (log_counter++ % 500 == 0)
        { // 每500次记录一次，避免刷屏
            std::stringstream ss;
            ss << "等待所有轴就绪: ";
            for (size_t i = 0; i < servo_axes_.size(); ++i)
            {
                AxisState state = servo_axes_[i]->get_current_state();
                if (state != AxisState::READY)
                {
                    ss << servo_axes_[i]->get_name() << "=" << static_cast<int>(state) << " ";
                }
            }
            RCLCPP_DEBUG(this->get_logger(), "%s", ss.str().c_str());
            log_counter = 0;
        }
    }

    // 更新手自动记忆（用于下次边沿检测）
    // 注：last_all_axes_ready_ 已在上方各分支内按"暂停抑制"后的语义更新，不能在这里无条件覆盖。
    last_manual_auto_state_ = current_manual_auto_state;
#endif
    // 发布IO状态
    publish_io_status();
}

void EthercatNode::publish_io_status()
{
    if (node_shutting_down_.load() || !rclcpp::ok())
    {
        return;
    }

    // 节流：/io_status固定约10Hz发布（与原IO线程100ms周期一致）。
    // 主循环提速后若不限频，HMI的Qt事件队列会被刷爆导致界面"无响应"
    static auto last_publish_time = std::chrono::steady_clock::now() - std::chrono::milliseconds(100);
    auto now = std::chrono::steady_clock::now();
    if (now - last_publish_time < std::chrono::milliseconds(100)) {
        return;
    }
    last_publish_time = now;

    auto msg = std_msgs::msg::String();
    std::stringstream ss;

    pthread_mutex_lock(&io_mutex_);

    // 读取当前DI和DO状态
    DI_Interface di = current_di_status_;
    DO_Interface do_status = get_current_do_state();

    // 发布编号约定（契约 C5）：DI{n}/DO{n} 的 n = 设备位偏移（= 数组下标），
    // 与 io_config.yaml 中 di_signals/do_signals 的书写顺序（按 address 升序）严格一致。
    // HMI 与下游都按「第 n 个 token ↔ di_signals[n]」对位，因此这里直接用 raw[] 遍历，
    // 不再手写字段指针数组（那样会把语义与位置重新绑死）。
    ss << "DI状态: ";
    for (int i = 0; i < di.count; ++i) {
        ss << "DI" << (i < 10 ? "0" : "") << i << ":" << (di.raw[i] ? "1" : "0");
        if (i + 1 < di.count) {
            ss << ",";
        }
    }

    ss << " | DO状态: ";

    for (int i = 0; i < do_status.count; ++i) {
        ss << "DO" << (i < 10 ? "0" : "") << i << ":" << (do_status.raw[i] ? "1" : "0");
        if (i + 1 < do_status.count) {
            ss << ",";
        }
    }

    pthread_mutex_unlock(&io_mutex_);

    msg.data = ss.str();
    io_status_pub_->publish(msg);
}

// 方案A：IO监控线程已移除
// 原线程逻辑（读DI -> handle_io_signals，100ms周期）已合并到main.cpp主循环，
// 由主线程统一执行，消除主线程与IO线程对灯光状态的并发访问。

void EthercatNode::monitor_di_changes(const DI_Interface &current_di)
{
    static DI_Interface previous_di = {0};
    static bool first = true;

    // 只监控角色表里的语义位，与具体点位顺序解耦；
    // 想排查某个通用点位（如气压），请直接看 /io_status 或按位偏移自查，不要再往这里加硬编码。
    if (first)
    {
        first = false;
        RCLCPP_INFO(this->get_logger(),
                    "[DI] 初始状态: 急停(正常)=%d 启动=%d 复位=%d 暂停=%d",
                    current_di.emergency_stop ? 1 : 0,
                    current_di.start_button ? 1 : 0,
                    current_di.reset_button ? 1 : 0,
                    current_di.pause_button ? 1 : 0);
    }

    const struct { const char *label; bool cur; bool prev; } watch[] = {
        {"急停(正常)", current_di.emergency_stop, previous_di.emergency_stop},
        {"启动",       current_di.start_button,   previous_di.start_button},
        {"复位",       current_di.reset_button,   previous_di.reset_button},
        {"暂停",       current_di.pause_button,   previous_di.pause_button},
    };

    for (const auto &w : watch)
    {
        if (w.cur != w.prev)
        {
            RCLCPP_INFO(this->get_logger(), "[DI] %s 状态变化: %s",
                        w.label, w.cur ? "HIGH" : "LOW");
        }
    }

    // 更新前一次状态
    previous_di = current_di;
}

void EthercatNode::handle_do_control(const std_msgs::msg::String::SharedPtr msg)
{
    if (node_shutting_down_.load() || !rclcpp::ok())
    {
        return;
    }

    std::string command = msg->data;
    RCLCPP_INFO(this->get_logger(), "收到DO控制命令: %s", command.c_str());

    // 解析命令
    DOControlCommand do_cmd;
    if (!parse_do_control_command(command, do_cmd))
    {
        print_error("DO控制命令解析失败: " + command);
        return;
    }

    // 执行DO控制
    int do_address = std::stoi(do_cmd.do_address);

    // 禁止通过外部命令修改"受保护"的 DO：
    // 缺省 = 三个按钮灯角色对应的位（由灯光控制器独占），可用 safety.protected_do 覆盖
    const int do_off = do_address - g_do_base_address;
    if (do_off >= 0 && do_off < IO_MAX_SIGNALS && g_do_protected[do_off])
    {
        RCLCPP_WARN(this->get_logger(), "拒绝外部修改受保护的DO%d（由灯光控制器独占控制）", do_address);
        return;
    }

    int result = write_single_do_signal(do_address, do_cmd.state);

    if (result == 1)
    {
        RCLCPP_INFO(this->get_logger(), "DO控制成功: %s -> %s",
                    do_cmd.do_address.c_str(),
                    do_cmd.state ? "true" : "false");

        // 发布状态更新
        publish_io_status();
    }
    else
    {
        print_error("DO控制失败: " + command + ", 错误码: " + std::to_string(result));
    }
}

// 虚拟IO指令处理
// 方案一：统一事件入口——虚拟按钮只设置虚拟按键电平，
// 由主循环与实体DI合并后统一走update_button_lights状态机，
// 灯效与系统动作与实体按钮完全一致（急停为点击切换锁存，再按一次解除）
void EthercatNode::handle_virtual_io(const std_msgs::msg::String::SharedPtr msg) {
    std::string command = msg->data;

    if (command == "virtual_start_down") {
        set_virtual_button(VBTN_START, true);
        RCLCPP_INFO(this->get_logger(), "[VirtualIO] 虚拟启动 按下");
    }
    else if (command == "virtual_start_up") {
        set_virtual_button(VBTN_START, false);
    }
    else if (command == "virtual_reset_down") {
        set_virtual_button(VBTN_RESET, true);
        RCLCPP_INFO(this->get_logger(), "[VirtualIO] 虚拟复位 按下（按住3秒确认）");
    }
    else if (command == "virtual_reset_up") {
        set_virtual_button(VBTN_RESET, false);
    }
    else if (command == "virtual_pause_down") {
        set_virtual_button(VBTN_PAUSE, true);
        RCLCPP_INFO(this->get_logger(), "[VirtualIO] 虚拟暂停 按下");
    }
    else if (command == "virtual_pause_up") {
        set_virtual_button(VBTN_PAUSE, false);
    }
    else if (command == "virtual_estop") {
        // 硬件急停未复位时忽略虚拟急停切换：
        //   此时系统已在安全停机态，软件锁存没有额外收益，反而会制造
        //   "硬件复位后必须到 HMI 点一次解除" 的隐性依赖，并让界面与实际相反
        //   （HMI 会因 |ESTOP 显示"点击解除"，点了却无效——这是刻意为之，避免误显示为已解除）。
        if (g_hw_estop_active.load()) {
            RCLCPP_WARN(this->get_logger(),
                        "[VirtualIO] 硬件急停未复位，忽略虚拟急停切换");
            return;
        }
        // 点击切换锁存：再按一次解除
        toggle_virtual_estop();
        RCLCPP_INFO(this->get_logger(), "[VirtualIO] 虚拟急停 %s",
                    is_virtual_estop_active() ? "激活（再按一次解除）" : "解除");
    }
    else {
        RCLCPP_WARN(this->get_logger(), "[VirtualIO] 未知命令: %s", command.c_str());
    }
}

// 添加点动速度设置处理函数
void EthercatNode::handle_jog_speed_command(const std_msgs::msg::String::SharedPtr msg)
{
    if (node_shutting_down_.load() || !rclcpp::ok())
    {
        return;
    }

    std::string command = msg->data;
    RCLCPP_INFO(this->get_logger(), "收到点动速度设置命令: %s", command.c_str());

    std::string axis_name;
    double speed;

    if (!parse_jog_speed_command(command, axis_name, speed))
    {
        print_error("点动速度命令解析失败: " + command);
        return;
    }

    // 查找对应的轴并设置速度
    bool axis_found = false;
    for (auto &axis : servo_axes_)
    {
        if (axis->get_name() == axis_name)
        {
            if (axis->set_jog_speed(speed))
            {
                RCLCPP_INFO(this->get_logger(), "成功设置轴 %s 的点动速度为: %.1f",
                            axis_name.c_str(), speed);

                // 发布系统状态
                auto status_msg = std_msgs::msg::String();
                status_msg.data = "轴 " + axis_name + " 点动速度设置为: " + std::to_string(speed);
                system_status_pub_->publish(status_msg);
            }
            else
            {
                print_error("设置轴 " + axis_name + " 的点动速度失败");
            }
            axis_found = true;
            break;
        }
    }

    if (!axis_found)
    {
        print_error("未找到轴: " + axis_name);
    }
}

// 添加点动速度解析函数
bool EthercatNode::parse_jog_speed_command(const std::string &command, std::string &axis_name, double &speed)
{
    // 格式: "axis_name:speed" 例如: "axis1_1:30.5"
    size_t colon_pos = command.find(':');
    if (colon_pos == std::string::npos)
    {
        print_error("无效命令格式，应为 '轴名:速度'");
        return false;
    }

    axis_name = command.substr(0, colon_pos);
    std::string speed_str = command.substr(colon_pos + 1);

    // 去除空格
    axis_name.erase(0, axis_name.find_first_not_of(" \t"));
    axis_name.erase(axis_name.find_last_not_of(" \t") + 1);
    speed_str.erase(0, speed_str.find_first_not_of(" \t"));
    speed_str.erase(speed_str.find_last_not_of(" \t") + 1);

    if (axis_name.empty() || speed_str.empty())
    {
        print_error("轴名或速度值为空");
        return false;
    }

    try
    {
        speed = std::stod(speed_str);
        return true;
    }
    catch (const std::exception &e)
    {
        print_error("速度值转换失败: " + speed_str + ", 错误: " + e.what());
        return false;
    }
}

// 检查所有轴是否都在自动模式
bool EthercatNode::are_all_axes_in_auto_mode()
{
    if (servo_axes_.empty())
    {
        return false; // 轴未初始化
    }

    for (const auto &axis : servo_axes_)
    {
        if (axis->get_current_state() != AxisState::AUTO_MODE)
        {
            return false; // 有轴不在自动模式
        }
    }
    return true; // 所有轴都在自动模式
}

// void EthercatNode::print_warning(const std::string& message) {
// bool has_fault = false;

// // 检查所有轴故障
// for (auto& axis : servo_axes_) {
//     if (axis->has_fault()) {
//         uint16_t fault_code = axis->get_fault_code();
//         std::string axis_name = axis->get_name();

//         if (has_fault) ss << ",";
//         has_fault = true;

//         ss << axis_name << ":0x" << std::hex << std::setw(4) << std::setfill('0') << fault_code;

//         // 同时通过故障管理器记录
//         fault_manager_->add_axis_fault(axis_name, fault_code, "驱动器故障");
//     }
// }

// // 如果还有其他故障源，也在这里添加

// std_msgs::msg::String msg;
// if (has_fault) {
//     std::string fault_str = ss.str();
//     msg.data = fault_str;
// } else {
//     msg.data = "0";
// }

// fault_code_pub_->publish(msg); 110ms发布周期，减少日志频率避免刷屏

// if (node_shutting_down_.load() || !rclcpp::ok()) {
//     return;
// }

// auto msg = std_msgs::msg::String();
// std::stringstream ss;

// bool has_fault = false;

// // 检查所有轴的故障状态
// for (auto& axis : servo_axes_) {
//     // 获取轴的错误代码
//     uint16_t error_code = axis->get_error_code();

//     // 检查是否处于故障状态
//     if (axis->get_current_state() == AxisState::FAULT) {
//         // 只有故障状态且错误码非0才发布
//         if (error_code != 0) {
//             ss << axis->get_name() << ":0x" << std::hex << error_code << ",";
//             has_fault = true;

//             // 记录故障日志（减少频率避免刷屏）
//             static std::unordered_map<std::string, uint16_t> last_error_codes;
//             uint16_t last_code = last_error_codes[axis->get_name()];

//             if (error_code != last_code) {
//                 RCLCPP_WARN(this->get_logger(),
//                            "检测到轴故障: %s, 错误码: 0x%04X",
//                            axis->get_name().c_str(), error_code);
//                 last_error_codes[axis->get_name()] = error_code;
//             }
//         }
//     }
// }

// if (has_fault) {
//     std::string fault_str = ss.str();
//     // 移除最后一个逗号
//     if (!fault_str.empty() && fault_str.back() == ',') {
//         fault_str.pop_back();
//     }
//     msg.data = fault_str;
// } else {
//     // 无故障时发布"0"
//     msg.data = "0";
// }

// fault_code_pub_->publish(msg);

// // 减少日志频率，避免刷屏
// static int log_counter = 0;
// if (log_counter++ % 50 == 0) {  // 每5秒记录一次（假设100ms发布周期）
//     if (has_fault) {
//         RCLCPP_DEBUG(this->get_logger(), "发布故障状态: %s", msg.data.c_str());
//     } else {
//         RCLCPP_DEBUG(this->get_logger(), "系统正常，无故障");
//     }
//     log_counter = 0;
// }
// }

void EthercatNode::print_warning(const std::string &message)
{
    RCLCPP_WARN(this->get_logger(), "%s", message.c_str());

    if (fault_manager_)
    {
        fault_manager_->add_system_warning(message);
    }
}

void EthercatNode::print_error(const std::string &message)
{
    RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());

    if (fault_manager_)
    {
        fault_manager_->add_system_error(message);
    }
}
