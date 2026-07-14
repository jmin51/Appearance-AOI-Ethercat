#include "ros2_bridge.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

Ros2Bridge::Ros2Bridge(QObject *parent)
    : QObject(parent), Node("qt_hmi")
{
    // 创建5个发布器 — topic 名称与 control_commander.py 完全一致
    control_pub_       = this->create_publisher<std_msgs::msg::String>("/control_command", 10);
    jog_pub_           = this->create_publisher<std_msgs::msg::String>("/jog_command", 10);
    displacement_pub_  = this->create_publisher<std_msgs::msg::String>("/displacement_command", 10);
    angle_pub_         = this->create_publisher<std_msgs::msg::String>("/angle_command", 10);
    io_control_pub_    = this->create_publisher<std_msgs::msg::String>("/do_control", 10);
    virtual_io_pub_    = this->create_publisher<std_msgs::msg::String>("/virtual_io", 10);

    // 订阅 /system_status — 接收控制器反馈的聚合系统状态
    system_status_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/system_status", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            onSystemStatusReceived(msg);
        });

    // 订阅 /joint_states — 接收各轴实时位置
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            onJointStatesReceived(msg);
        });

    // 从 YAML 加载轴配置
    try {
        std::string configPath = ament_index_cpp::get_package_share_directory("ethercat_controller")
                                 + "/config/axes_config.yaml";
        if (!loadAxesFromYaml(configPath)) {
            emit logMessage(QString("[ROS2] WARNING: 轴配置加载失败, 轴数为0"));
        }
    } catch (const std::exception &e) {
        emit logMessage(QString("[ROS2] ERROR: 读取轴配置文件异常: %1").arg(e.what()));
    }

    // 从 YAML 加载 IO 配置
    bool io_loaded = false;
    try {
        std::string ioConfigPath = ament_index_cpp::get_package_share_directory("ethercat_controller")
                                   + "/config/io_config.yaml";
        io_loaded = loadIoConfigFromYaml(ioConfigPath);
        if (io_loaded) {
            emit logMessage(QString("[ROS2] IO配置加载成功: %1 DI + %2 DO")
                                .arg(di_signals_.size()).arg(do_signals_.size()));
        }
    } catch (const std::exception &e) {
        emit logMessage(QString("[ROS2] ERROR: 读取IO配置文件异常: %1").arg(e.what()));
    }

    // 初始化 IO 状态 — DI 和 DO 分开管理
    for (const auto &di : di_signals_) {
        di_status_[di.address] = 0;
    }
    for (const auto &dout : do_signals_) {
        do_status_[dout.address] = 0;
    }

    // 订阅 /io_status — 接收控制器发布的完整 DI/DO 状态
    if (io_loaded) {
        io_status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/io_status", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                onIoStatusReceived(msg);
            });
    }

    emit logMessage(QString("[ROS2] qt_hmi node started, %1 axes, 5 publishers ready").arg(num_axes_));
}

// ————— YAML 轴配置加载 —————
bool Ros2Bridge::loadAxesFromYaml(const std::string &yamlPath)
{
    YAML::Node config;
    try {
        config = YAML::LoadFile(yamlPath);
    } catch (const YAML::Exception &e) {
        emit logMessage(QString("[ROS2] YAML parse error: %1").arg(e.what()));
        return false;
    }

    if (!config["servo_axes"]) {
        emit logMessage("[ROS2] YAML missing 'servo_axes' node");
        return false;
    }

    const YAML::Node &axes = config["servo_axes"];
    if (!axes.IsSequence()) {
        emit logMessage("[ROS2] 'servo_axes' must be a sequence");
        return false;
    }

    for (size_t i = 0; i < axes.size(); ++i) {
        const YAML::Node &a = axes[i];
        try {
            std::string name         = a["name"].as<std::string>();
            std::string calc_mode    = a["calc_mode"] ? a["calc_mode"].as<std::string>() : "DISPLACEMENT";
            std::string display_name = a["display_name"] ? a["display_name"].as<std::string>() : name;

            axis_names_.push_back(name);
            axis_calc_modes_.push_back(calc_mode);
            axis_display_names_.push_back(display_name);
        } catch (const YAML::Exception &e) {
            emit logMessage(QString("[ROS2] YAML axis[%1] parse error: %2").arg(i).arg(e.what()));
            return false;
        }
    }

    num_axes_ = static_cast<int>(axis_names_.size());

    emit logMessage(QString("[ROS2] Loaded %1 axes from %2").arg(num_axes_).arg(yamlPath.c_str()));
    return num_axes_ > 0;
}

// ————— YAML IO 配置加载 —————
bool Ros2Bridge::loadIoConfigFromYaml(const std::string &yamlPath)
{
    YAML::Node config;
    try {
        config = YAML::LoadFile(yamlPath);
    } catch (const YAML::Exception &e) {
        emit logMessage(QString("[ROS2] IO YAML parse error: %1").arg(e.what()));
        return false;
    }

    // 解析 DI 信号
    if (config["di_signals"]) {
        auto sig_list = config["di_signals"];
        for (size_t i = 0; i < sig_list.size(); i++) {
            IoSignalInfo info;
            info.name    = sig_list[i]["name"].as<std::string>();
            info.desc    = sig_list[i]["desc"] ? sig_list[i]["desc"].as<std::string>() : info.name;
            info.address = std::to_string(sig_list[i]["address"].as<int>());
            info.is_input = true;
            di_signals_.push_back(info);
        }
    }

    // 解析 DO 信号
    if (config["do_signals"]) {
        auto sig_list = config["do_signals"];
        for (size_t i = 0; i < sig_list.size(); i++) {
            IoSignalInfo info;
            info.name    = sig_list[i]["name"].as<std::string>();
            info.desc    = sig_list[i]["desc"] ? sig_list[i]["desc"].as<std::string>() : info.name;
            info.address = std::to_string(sig_list[i]["address"].as<int>());
            info.is_input = false;
            do_signals_.push_back(info);
        }
    }

    return !di_signals_.empty() || !do_signals_.empty();
}

int Ros2Bridge::getAxisIndex(const std::string &name) const
{
    for (int i = 0; i < num_axes_; ++i) {
        if (axis_names_[i] == name)
            return i;
    }
    return -1;
}

// ————— 辅助 —————
void Ros2Bridge::publishString(
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub,
    const std::string &data)
{
    auto msg = std_msgs::msg::String();
    msg.data = data;
    pub->publish(msg);
}

// ————— 控制命令 —————
void Ros2Bridge::publishControlCommand(const std::string &command)
{
    publishString(control_pub_, command);
    emit logMessage(QString("[CMD] %1").arg(command.c_str()));
}

// ————— 点动命令 —————
void Ros2Bridge::publishJogCommand(const std::string &axis, const std::string &direction)
{
    publishString(jog_pub_, axis + ":" + direction);
    emit logMessage(QString("[JOG] %1:%2").arg(axis.c_str(), direction.c_str()));
}

void Ros2Bridge::publishJogStop(const std::string &axis)
{
    publishString(jog_pub_, axis + ":stop");
}

void Ros2Bridge::publishJogStopAll()
{
    for (int i = 0; i < num_axes_; ++i) {
        publishString(jog_pub_, axis_names_[i] + ":stop");
    }
    emit logMessage("[JOG] All axes stopped");
}

// ————— 位移命令 —————
void Ros2Bridge::publishDisplacement(const std::string &axis, double position_mm)
{
    std::ostringstream oss;
    oss << axis << ":" << std::fixed << std::setprecision(3) << position_mm;
    publishString(displacement_pub_, oss.str());
    emit logMessage(QString("[DISP] %1:%2 mm").arg(axis.c_str()).arg(position_mm, 0, 'f', 3));
}

// ————— 角度命令 —————
void Ros2Bridge::publishAngle(const std::string &axis, double angle_deg)
{
    double angle_rad = angle_deg * M_PI / 180.0;
    std::ostringstream oss;
    oss << axis << ":" << std::fixed << std::setprecision(6) << angle_rad;
    publishString(angle_pub_, oss.str());
    emit logMessage(QString("[ANGLE] %1:%2° (%3 rad)")
                        .arg(axis.c_str())
                        .arg(angle_deg, 0, 'f', 2)
                        .arg(angle_rad, 0, 'f', 4));
}

// ————— IO 控制 —————
void Ros2Bridge::publishIoControl(const std::string &address, int value)
{
    // 仅允许控制 DO 输出信号
    std::string desc = address;
    bool is_do = false;
    for (const auto &dout : do_signals_) {
        if (dout.address == address) {
            desc = dout.desc;
            is_do = true;
            break;
        }
    }
    if (!is_do) {
        emit logMessage(QString("[IO] 地址 M%1 不是DO输出信号，忽略控制").arg(address.c_str()));
        return;
    }

    do_status_[address] = value;
    publishString(io_control_pub_, address + ":" + std::to_string(value));
    emit logMessage(QString("[IO] M%1 %2 → %3")
                        .arg(address.c_str(), desc.c_str())
                        .arg(value ? "ON" : "OFF"));
    emit ioStatusUpdated(address, value);
}

// ————— 虚拟 IO —————
void Ros2Bridge::publishVirtualIo(const std::string &command)
{
    publishString(virtual_io_pub_, command);
    emit logMessage(QString("[VirtualIO] %1").arg(command.c_str()));
}

// ————— 系统状态回调 —————
void Ros2Bridge::onSystemStatusReceived(const std_msgs::msg::String::SharedPtr msg)
{
    QString status = QString::fromStdString(msg->data);
    emit systemStatusChanged(status);
}

// ————— 关节状态回调 —————
void Ros2Bridge::onJointStatesReceived(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
    //     "收到 /joint_states, %zu 个轴", msg->name.size());
    emit jointStatesUpdated(msg->name, msg->position);
}

// ————— IO 状态回调 (来自 /io_status) —————
void Ros2Bridge::onIoStatusReceived(const std_msgs::msg::String::SharedPtr msg)
{
    // 消息格式: "DI状态: DI00:1,DI01:0,... | DO状态: DO00:0,DO01:1,..."
    const std::string &data = msg->data;

    // 分割 DI 和 DO 部分
    size_t sep = data.find(" | ");
    std::string di_part, do_part;
    if (sep != std::string::npos) {
        di_part = data.substr(0, sep);
        do_part = data.substr(sep + 3);
    } else {
        // 兼容仅有 DI 或仅有 DO 的情况
        if (data.find("DI状态:") != std::string::npos)
            di_part = data;
        else if (data.find("DO状态:") != std::string::npos)
            do_part = data;
    }

    // 解析 DI 状态: "DI状态: DI00:1,DI01:0,..."
    if (!di_part.empty()) {
        size_t colon = di_part.find(':');
        if (colon != std::string::npos) {
            std::string values = di_part.substr(colon + 1);
            // 去掉前后空格
            size_t start = values.find_first_not_of(" \t");
            size_t end   = values.find_last_not_of(" \t");
            if (start != std::string::npos)
                values = values.substr(start, end - start + 1);

            // 按逗号分割各信号
            std::stringstream ss(values);
            std::string token;
            int idx = 0;
            while (std::getline(ss, token, ',')) {
                // token 格式: "DI00:1" 或 "DI01:0"
                size_t c = token.find(':');
                if (c != std::string::npos) {
                    int val = (token.substr(c + 1) == "1") ? 1 : 0;
                    // 通过索引映射到 DI 信号列表
                    if (idx < (int)di_signals_.size()) {
                        std::string addr = di_signals_[idx].address;
                        di_status_[addr] = val;
                        emit diStatusUpdated(addr, val);
                    }
                }
                idx++;
            }
        }
    }

    // 解析 DO 状态: "DO状态: DO00:0,DO01:1,..."
    if (!do_part.empty()) {
        size_t colon = do_part.find(':');
        if (colon != std::string::npos) {
            std::string values = do_part.substr(colon + 1);
            size_t start = values.find_first_not_of(" \t");
            size_t end   = values.find_last_not_of(" \t");
            if (start != std::string::npos)
                values = values.substr(start, end - start + 1);

            std::stringstream ss(values);
            std::string token;
            int idx = 0;
            while (std::getline(ss, token, ',')) {
                size_t c = token.find(':');
                if (c != std::string::npos) {
                    int val = (token.substr(c + 1) == "1") ? 1 : 0;
                    if (idx < (int)do_signals_.size()) {
                        std::string addr = do_signals_[idx].address;
                        do_status_[addr] = val;
                        emit ioStatusUpdated(addr, val);
                    }
                }
                idx++;
            }
        }
    }
}
