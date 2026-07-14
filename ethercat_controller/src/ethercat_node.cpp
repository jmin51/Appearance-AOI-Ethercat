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

#define CONTROL_SOURCE_IO 0 // 1:使用IO控制手自动模式, 0:使用话题控制
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
std::atomic<bool> g_pause_button_pressed(false);
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

    dual_displacement_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/dual_displacement_command", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::String::SharedPtr msg)
        {
            handle_displacement_command(msg);
        });

    XYR_displacement_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/XYR_displacement_command", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::String::SharedPtr msg)
        {
            handle_displacement_command(msg);
        });
        
    // 添加角度控制订阅器 (rad)
    angle_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/angle_command", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::String::SharedPtr msg)
        {
            handle_angle_command(msg);
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
    if (all_fault || any_fault)   status = "FAULT";
    else if (all_ready)           status = "READY";
    else if (all_manual)          status = "MANUAL";
    else if (all_auto)            status = "AUTO";
    else if (all_stopped)         status = "STOPPED";
    else                          status = "INIT";

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
    stop_io_monitoring(); // 等待IO线程完全退出
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

            DriveBrand brand = ServoAxisFactory::string_to_brand(brand_str);
            AxisType axis_type = ServoAxisFactory::string_to_axis_type(axis_type_str);
            AxisCalcMode calc_mode = ServoAxisFactory::string_to_calc_mode(calc_mode_str);

            auto axis = ServoAxisFactory::create_servo_axis(
                brand, name, position, axis_type, product_code, gear_ratio, calc_mode);

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
                axis->set_auto_speed(auto_speed);
                RCLCPP_INFO(this->get_logger(), "轴 %s 自动速度设置为: %.1f",
                            name.c_str(), auto_speed);
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
        // 手动模式
        for (auto &axis : servo_axes_)
        {
            axis->start_manual_mode();
        }
        RCLCPP_INFO(this->get_logger(), "所有轴接收到手动模式启动命令");
    }
    else if (command == CMD_START_AUTO)
    {
        // 自动模式
        for (auto &axis : servo_axes_)
        {
            axis->start_auto_mode();
        }
        RCLCPP_INFO(this->get_logger(), "所有轴接收到自动模式启动命令");
    }
    else if (command == CMD_STOP)
    {
        // 停止模式
        for (auto &axis : servo_axes_)
        {
            axis->stop();
        }
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
        // 清除故障
        for (auto &axis : servo_axes_)
        {
            axis->clear_fault();
        }
        RCLCPP_INFO(this->get_logger(), "所有轴接收到清除故障命令");
    }
    else if (command == CMD_RESET)
    {
        // 重置
        for (auto &axis : servo_axes_)
        {
            axis->reset_axis();
        }
        RCLCPP_INFO(this->get_logger(), "所有轴接收到重置命令");
    }
    else if (command == "clear_all_faults")
    {
        // 清除所有系统故障和告警
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

void EthercatNode::handle_axis_command(size_t axis_index, double newTargetPosition)
{
    if (axis_index < servo_axes_.size())
    {
        servo_axes_[axis_index]->set_target_displacement(newTargetPosition);
        servo_axes_[axis_index]->set_displacement_updated(true);

        RCLCPP_DEBUG(this->get_logger(), "轴[%zu] 位移指令: %.4fmm",
                     axis_index, newTargetPosition);
        // // 详细状态检查
        // RCLCPP_INFO(this->get_logger(), "运行状态=%s, 当前状态=%d",
        //         axis->is_running() ? "是" : "否",
        //         static_cast<int>(axis->get_current_state()));
    }
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

        // 发送角度指令（度），轴侧 angle_to_pulses 将度转换为脉冲
        handle_axis_command(axis_index, angle_deg);
        RCLCPP_INFO(this->get_logger(), "轴 %s 角度指令: %.4f rad (%.2f°)",
                    axis_name.c_str(), angle_rad, angle_deg);
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

void EthercatNode::start_io_monitoring()
{
    if (io_running_.load())
    {
        RCLCPP_WARN(this->get_logger(), "IO监控线程已在运行");
        return;
    }

    // 从YAML加载IO配置
    std::string io_config_path = this->get_parameter("io_config_path").as_string();
    if (!load_io_config(io_config_path)) {
        print_error("加载IO配置失败，使用默认配置");
    }

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
            print_error("Modbus初始化失败");
            return;
        }
    }

    io_running_.store(true);

    // 创建IO监控线程
    if (pthread_create(&io_thread_, nullptr, io_monitor_thread, this) != 0)
    {
        print_error("创建IO监控线程失败");
        io_running_.store(false);
        cleanup_modbus_interface();
        return;
    }

    pthread_setname_np(io_thread_, "io-monitor");
    RCLCPP_INFO(this->get_logger(), "IO监控线程启动成功");
}

void EthercatNode::stop_io_monitoring()
{
    if (!io_running_.load())
    {
        return;
    }

    io_running_.store(false);

    if (io_thread_)
    {
        pthread_join(io_thread_, nullptr);
        io_thread_ = 0;
    }

    cleanup_modbus_interface();
    pthread_mutex_destroy(&io_mutex_);
    RCLCPP_INFO(this->get_logger(), "IO监控线程已停止");
}

void EthercatNode::handle_io_signals(DI_Interface di)
{
    // 发布IO状态到Python节点
    // publish_py_io_status(di);

    monitor_di_changes(di);
    pthread_mutex_lock(&io_mutex_);
    current_di_status_ = di;
    pthread_mutex_unlock(&io_mutex_);

    // 此处仅检测按钮状态并设置全局标志，不直接控制DO
    // AL states 0x08 (SAFE-OP) 且全部进入自动模式 - 通知灯光控制器熄灭复位灯，切换为绿灯闪烁
    if (master_state.al_states == 0x08 && are_all_axes_in_auto_mode())
    {
        notify_system_ready();
        // RCLCPP_INFO(this->get_logger(), "AL states=0x08 且全部轴进入自动模式，系统就绪");
    }

    // 新增：IO控制模式切换（仅在宏开关启用时生效）
#if CONTROL_SOURCE_IO
    bool current_manual_auto_state = di.manual_auto_button; // 当前按钮状态（DI04）

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

    // 如果所有轴都处于READY状态，尝试模式切换
    if (all_axes_ready)
    {
        // 仅在状态变化时打印日志（边沿检测）
        if (!last_all_axes_ready_ || last_manual_auto_state_ != current_manual_auto_state)
        {
            RCLCPP_INFO(this->get_logger(), "所有轴已就绪，准备模式切换。手自动按钮状态: %s",
                        current_manual_auto_state ? "自动" : "手动");

            // 触发模式切换命令
            std::string command = current_manual_auto_state ? CMD_START_AUTO : CMD_START_MANUAL;
            handle_control_command(command); // 通过统一命令处理

            RCLCPP_INFO(this->get_logger(), "已发送%s模式切换命令",
                        current_manual_auto_state ? "自动" : "手动");
        }
    }
    else
    {
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

    // 更新状态记忆（用于下次边沿检测）
    last_all_axes_ready_ = all_axes_ready;
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

    auto msg = std_msgs::msg::String();
    std::stringstream ss;

    pthread_mutex_lock(&io_mutex_);

    // 读取当前DI和DO状态
    DI_Interface di = current_di_status_;
    DO_Interface do_status = get_current_do_state();

    // 按照表格顺序发布DI状态 (0-15)
    ss << "DI状态: ";
    ss << "DI00:" << (di.start_button ? "1" : "0") << ",";           // M500 启动按钮
    ss << "DI01:" << (di.reset_button ? "1" : "0") << ",";           // M501 复位按钮
    ss << "DI02:" << (di.pause_button ? "1" : "0") << ",";           // M502 暂停按钮
    ss << "DI03:" << (di.manual_auto_button ? "1" : "0") << ",";     // M503 手自动按钮
    ss << "DI04:" << (di.emergency_stop ? "1" : "0") << ",";         // M504 急停按钮
    ss << "DI05:" << (di.air_supply ? "1" : "0") << ",";             // M505 气源输入
    ss << "DI06:" << (di.di_M506 ? "1" : "0") << ",";                // M506
    ss << "DI07:" << (di.di_M507 ? "1" : "0") << ",";                // M507
    ss << "DI08:" << (di.di_M508 ? "1" : "0") << ",";                // M508
    ss << "DI09:" << (di.di_M509 ? "1" : "0") << ",";                // M509
    ss << "DI10:" << (di.di_M510 ? "1" : "0") << ",";                // M510
    ss << "DI11:" << (di.di_M511 ? "1" : "0") << ",";                // M511
    ss << "DI12:" << (di.di_M512 ? "1" : "0") << ",";                // M512
    ss << "DI13:" << (di.di_M513 ? "1" : "0") << ",";                // M513
    ss << "DI14:" << (di.di_M514 ? "1" : "0") << ",";                // M514
    ss << "DI15:" << (di.di_M515 ? "1" : "0");                       // M515

    ss << " | DO状态: ";

    // 按照表格顺序发布DO状态 (0-15)
    ss << "DO00:" << (do_status.start_button_light ? "1" : "0") << ",";   // M800 启动按钮灯
    ss << "DO01:" << (do_status.reset_button_light ? "1" : "0") << ",";   // M801 复位按钮灯
    ss << "DO02:" << (do_status.pause_button_light ? "1" : "0") << ",";   // M802 暂停按钮灯
    ss << "DO03:" << (do_status.buzzer ? "1" : "0") << ",";               // M803 蜂鸣器
    ss << "DO04:" << (do_status.red_light ? "1" : "0") << ",";            // M804 三色红灯
    ss << "DO05:" << (do_status.yellow_light ? "1" : "0") << ",";         // M805 三色黄灯
    ss << "DO06:" << (do_status.green_light ? "1" : "0") << ",";          // M806 三色绿灯
    ss << "DO07:" << (do_status.do_M807 ? "1" : "0") << ",";              // M807
    ss << "DO08:" << (do_status.do_M808 ? "1" : "0") << ",";              // M808
    ss << "DO09:" << (do_status.do_M809 ? "1" : "0") << ",";              // M809
    ss << "DO10:" << (do_status.do_M810 ? "1" : "0") << ",";              // M810
    ss << "DO11:" << (do_status.do_M811 ? "1" : "0") << ",";              // M811
    ss << "DO12:" << (do_status.do_M812 ? "1" : "0") << ",";              // M812
    ss << "DO13:" << (do_status.do_M813 ? "1" : "0") << ",";              // M813
    ss << "DO14:" << (do_status.do_M814 ? "1" : "0") << ",";              // M814
    ss << "DO15:" << (do_status.do_M815 ? "1" : "0");                     // M815

    pthread_mutex_unlock(&io_mutex_);

    msg.data = ss.str();
    io_status_pub_->publish(msg);
}

// IO监控线程函数
void *io_monitor_thread(void *arg)
{
    EthercatNode *node = static_cast<EthercatNode *>(arg);
    time_t last_display = time(NULL);
    RCLCPP_INFO(node->get_logger(), "IO监控线程开始运行");

    // DI/DO 模块禁用警告（运行时检查）
    if (!is_di_module_enabled()) {
        static bool di_disabled_warned = false;
        if (!di_disabled_warned) {
            node->print_warning("DI模块已禁用");
            di_disabled_warned = true;
        }
    }

    if (!is_do_module_enabled()) {
        static bool do_disabled_warned = false;
        if (!do_disabled_warned) {
            node->print_warning("DO模块已禁用");
            do_disabled_warned = true;
        }
    }

    while (node->is_io_running() && !g_should_exit.load())
    {
        // 读取DI信号（如果启用）
        DI_Interface di;
        di = read_all_di_signals();

        // 获取当前DO状态
        // DO_Interface do_control = get_current_do_state();

        // 处理IO信号
        node->handle_io_signals(di);

        usleep(100000); // 100ms刷新周期
    }

    RCLCPP_INFO(node->get_logger(), "IO监控线程退出");
    return nullptr;
}

void EthercatNode::monitor_di_changes(const DI_Interface &current_di)
{
    static DI_Interface previous_di = {0};

    // 检查每个DI信号的变化
    std::vector<std::pair<std::string, bool>> changes;

    if (current_di.di_M511 != previous_di.di_M511)
    {
        changes.push_back({"DI11", current_di.di_M511});
    }
    if (current_di.di_M512 != previous_di.di_M512)
    {
        changes.push_back({"DI12", current_di.di_M512});
    }
    if (current_di.di_M513 != previous_di.di_M513)
    {
        changes.push_back({"DI13", current_di.di_M513});
    }
    if (current_di.di_M514 != previous_di.di_M514)
    {
        changes.push_back({"DI14", current_di.di_M514});
    }

    // 打印变化
    for (const auto &change : changes)
    {
        RCLCPP_INFO(this->get_logger(), "%s 状态变化: %s",
                    change.first.c_str(),
                    change.second ? "HIGH" : "LOW");
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

    // 禁止通过外部命令修改按钮灯状态（前3个DO），由handle_io_signals独占控制
    if (do_address >= g_do_base_address && do_address <= g_do_base_address + 2)
    {
        RCLCPP_WARN(this->get_logger(), "拒绝外部修改按钮灯DO%d，按钮灯由IO模块独占控制", do_address);
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
void EthercatNode::handle_virtual_io(const std_msgs::msg::String::SharedPtr msg) {
    std::string command = msg->data;

    if (command == "virtual_start") {
        RCLCPP_INFO(this->get_logger(), "[VirtualIO] 虚拟启动");
        g_start_button_pressed.store(true);
    }
    else if (command == "virtual_estop") {
        RCLCPP_INFO(this->get_logger(), "[VirtualIO] 虚拟急停");
        g_full_shutdown_requested.store(true);
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
