#include "ros2_bridge.hpp"

#include <QStringList>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <vector>

#include <fcntl.h>
#include <unistd.h>

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

// ============================================================================
// 文件读写辅助（方案C：直接改 axes_config.yaml，必须保证原子性与可回滚）
// ============================================================================
namespace {

// 判断文件是否存在
bool fileExists(const std::string &path)
{
    std::ifstream f(path, std::ios::binary);
    return f.good();
}

// 读取整个文件（二进制，保留原换行风格 CRLF/LF）
bool readFileToString(const std::string &path, std::string &out)
{
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs) return false;
    out.assign(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());
    return true;
}

// 二进制拷贝文件
bool copyFileTo(const std::string &src, const std::string &dst)
{
    std::string data;
    if (!readFileToString(src, data)) return false;
    std::ofstream ofs(dst, std::ios::binary | std::ios::trunc);
    if (!ofs) return false;
    ofs.write(data.data(), static_cast<std::streamsize>(data.size()));
    return ofs.good();
}

// 原子写：临时文件 + fsync + rename
// 避免控制器在 init_axes() 读盘时看到写了一半的文件（会导致 0 轴 → 域数据为空 → 安全关闭）
bool atomicWriteFile(const std::string &path, const std::string &data)
{
    const std::string tmp = path + ".tmp";
    {
        std::ofstream ofs(tmp, std::ios::binary | std::ios::trunc);
        if (!ofs) return false;
        ofs.write(data.data(), static_cast<std::streamsize>(data.size()));
        ofs.flush();
        if (!ofs.good()) return false;
    }
    int fd = ::open(tmp.c_str(), O_RDONLY);
    if (fd >= 0) { ::fsync(fd); ::close(fd); }

    if (std::rename(tmp.c_str(), path.c_str()) != 0) {
        std::remove(tmp.c_str());
        return false;
    }

    // 同步目录项，确保 rename 本身落盘
    const size_t slash = path.find_last_of('/');
    if (slash != std::string::npos) {
        const std::string dir = path.substr(0, slash);
        int dfd = ::open(dir.c_str(), O_RDONLY | O_DIRECTORY);
        if (dfd >= 0) { ::fsync(dfd); ::close(dfd); }
    }
    return true;
}

// 去掉行尾 \r（仅用于匹配判断，不改动原始内容）
std::string stripCr(const std::string &s)
{
    if (!s.empty() && s.back() == '\r') return s.substr(0, s.size() - 1);
    return s;
}

// 行首空格数
size_t leadingSpaces(const std::string &s)
{
    size_t n = 0;
    while (n < s.size() && s[n] == ' ') ++n;
    return n;
}

// 解析 name 行，输出轴名（形如 `    name: "axis1"`）
bool parseNameLine(const std::string &line, std::string &out)
{
    const std::string t = stripCr(line);
    const size_t colon = t.find(':');
    if (colon == std::string::npos) return false;
    const size_t ks = t.find_first_not_of(" \t");
    if (ks == std::string::npos) return false;
    if (t.compare(ks, colon - ks, "name") != 0) return false;

    const std::string val = t.substr(colon + 1);
    const size_t vs = val.find_first_not_of(" \t\"");
    const size_t ve = val.find_last_not_of(" \t\"");
    if (vs == std::string::npos || ve == std::string::npos || ve < vs) return false;
    out = val.substr(vs, ve - vs + 1);
    return true;
}

// 判断去缩进后的行是否以指定 key 开头（缩进必须等于 expectIndent）
// 注释行以 '#' 开头，天然不会匹配
bool lineHasKeyAtIndent(const std::string &line, const std::string &key, size_t expectIndent)
{
    const std::string t = stripCr(line);
    const size_t ind = leadingSpaces(t);
    if (ind != expectIndent) return false;
    return t.compare(ind, key.size(), key) == 0;
}

// 把原始文本按 '\n' 切分（保留每行末尾的 '\r'，回写时原样还原换行风格）
std::vector<std::string> splitLines(const std::string &raw)
{
    std::vector<std::string> lines;
    std::string cur;
    for (char c : raw) {
        if (c == '\n') { lines.push_back(cur); cur.clear(); }
        else           { cur.push_back(c); }
    }
    if (!cur.empty()) lines.push_back(cur);
    return lines;
}

std::string joinLines(const std::vector<std::string> &lines, bool trailingNewline)
{
    std::string out;
    for (size_t i = 0; i < lines.size(); ++i) {
        out += lines[i];
        if (i + 1 < lines.size() || trailingNewline) out += "\n";
    }
    return out;
}

} // namespace

Ros2Bridge::Ros2Bridge(QObject *parent)
    : QObject(parent), Node("qt_hmi")
{
    // 创建发布器 — topic 名称与 ethercat_controller 的订阅完全一致
    control_pub_       = this->create_publisher<std_msgs::msg::String>("/control_command", 10);
    jog_pub_           = this->create_publisher<std_msgs::msg::String>("/jog_command", 10);
    displacement_pub_  = this->create_publisher<std_msgs::msg::String>("/displacement_command", 10);
    angle_pub_         = this->create_publisher<std_msgs::msg::String>("/angle_command", 10);
    io_control_pub_    = this->create_publisher<std_msgs::msg::String>("/do_control", 10);
    virtual_io_pub_    = this->create_publisher<std_msgs::msg::String>("/virtual_io", 10);
    // 速度百分比指令（"默认"速度仍走上面两个话题；选择百分比时走这两个）
    displacement_pct_pub_ = this->create_publisher<std_msgs::msg::String>("/displacement_command_pct", 10);
    angle_pct_pub_        = this->create_publisher<std_msgs::msg::String>("/angle_command_pct", 10);

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
    // 重要：控制器与 HMI 必须加载【同一份】io_config.yaml —— 发布编号 DI{n}/DO{n} 的 n
    //       就是设备位偏移，两端都按「第 n 个信号」对位。这里把"实际路径 + 点数 + 地址范围"
    //       打进日志，与控制器启动日志逐项比对即可发现"两边用了不同文件"这类隐性错位。
    bool io_loaded = false;
    try {
        std::string ioConfigPath = ament_index_cpp::get_package_share_directory("ethercat_controller")
                                   + "/config/io_config.yaml";
        io_loaded = loadIoConfigFromYaml(ioConfigPath);
        if (io_loaded) {
            emit logMessage(QString("[ROS2] IO配置加载成功: %1 DI + %2 DO (path: %3)")
                                .arg(di_signals_.size()).arg(do_signals_.size())
                                .arg(QString::fromStdString(ioConfigPath)));
            if (!di_signals_.empty()) {
                emit logMessage(QString("[ROS2] DI 地址范围: M%1 ~ M%2")
                                    .arg(QString::fromStdString(di_signals_.front().address))
                                    .arg(QString::fromStdString(di_signals_.back().address)));
            }
            if (!do_signals_.empty()) {
                emit logMessage(QString("[ROS2] DO 地址范围: M%1 ~ M%2")
                                    .arg(QString::fromStdString(do_signals_.front().address))
                                    .arg(QString::fromStdString(do_signals_.back().address)));
            }
        } else {
            emit logMessage(QString("[ROS2] ERROR: IO配置加载失败 (path: %1)")
                                .arg(QString::fromStdString(ioConfigPath)));
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

    // 记录路径并清空旧数据（reloadAxesConfig 会重复调用本函数）
    axes_config_path_ = yamlPath;
    axis_names_.clear();
    axis_calc_modes_.clear();
    axis_display_names_.clear();
    axis_homing_values_.clear();
    axis_homing_capable_.clear();
    num_axes_ = 0;

    for (size_t i = 0; i < axes.size(); ++i) {
        const YAML::Node &a = axes[i];
        try {
            std::string name         = a["name"].as<std::string>();
            std::string calc_mode    = a["calc_mode"] ? a["calc_mode"].as<std::string>() : "DISPLACEMENT";
            std::string display_name = a["display_name"] ? a["display_name"].as<std::string>() : name;

            // 回原标记（初始值）：operation_mode == homing/HOMING/hm/HM
            // 与 ethercat_controller 的解析规则保持一致
            bool homing = false;
            if (a["operation_mode"]) {
                const std::string om = a["operation_mode"].as<std::string>();
                homing = (om == "homing" || om == "HOMING" || om == "hm" || om == "HM");
            }

            // 能力判断：仅汇川/雷赛伺服列入回原白名单
            bool capable = false;
            if (a["brand"]) {
                const std::string br = a["brand"].as<std::string>();
                capable = (br == "HUICHUAN" || br == "LEISAI");
            }

            axis_names_.push_back(name);
            axis_calc_modes_.push_back(calc_mode);
            axis_display_names_.push_back(display_name);
            axis_homing_values_.push_back(homing);
            axis_homing_capable_.push_back(capable);
        } catch (const YAML::Exception &e) {
            emit logMessage(QString("[ROS2] YAML axis[%1] parse error: %2").arg(i).arg(e.what()));
            return false;
        }
    }

    num_axes_ = static_cast<int>(axis_names_.size());

    // 打印当前回原轴清单，便于与控制器日志对账
    QStringList homing_axes;
    for (int i = 0; i < num_axes_; ++i) {
        if (axis_homing_values_[i]) homing_axes << QString::fromStdString(axis_names_[i]);
    }
    emit logMessage(QString("[ROS2] Loaded %1 axes from %2 (回原轴: %3)")
                        .arg(num_axes_).arg(yamlPath.c_str())
                        .arg(homing_axes.isEmpty() ? QString("无") : homing_axes.join(",")));
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

    // 重新加载时先清空，避免重复追加
    di_signals_.clear();
    do_signals_.clear();

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

    // 防御性截断：控制器侧 IO_MAX_SIGNALS = 48，超出即说明两边 yaml 不一致
    const size_t kIoMaxSignals = 48;
    if (di_signals_.size() > kIoMaxSignals) {
        emit logMessage(QString("[ROS2] WARN: DI 条数 %1 超过上限 %2，已截断"
                                "（请确认与控制器加载的是同一份 io_config.yaml）")
                            .arg(di_signals_.size()).arg((int)kIoMaxSignals));
        di_signals_.resize(kIoMaxSignals);
    }
    if (do_signals_.size() > kIoMaxSignals) {
        emit logMessage(QString("[ROS2] WARN: DO 条数 %1 超过上限 %2，已截断"
                                "（请确认与控制器加载的是同一份 io_config.yaml）")
                            .arg(do_signals_.size()).arg((int)kIoMaxSignals));
        do_signals_.resize(kIoMaxSignals);
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

// ————— 速度百分比指令 —————
// 消息格式与控制器 /displacement_command_pct 一致: "轴名:目标mm:整数百分比"
void Ros2Bridge::publishDisplacementPct(const std::string &axis, double position_mm, int percent)
{
    std::ostringstream oss;
    oss << axis << ":" << std::fixed << std::setprecision(3) << position_mm
        << ":" << percent;   // 控制器要求纯整数百分比(5~100)，不能带小数点
    publishString(displacement_pct_pub_, oss.str());
    emit logMessage(QString("[DISP pct=%1] %2:%3 mm")
                        .arg(percent)
                        .arg(axis.c_str())
                        .arg(position_mm, 0, 'f', 3));
}

// 消息格式与控制器 /angle_command_pct 一致: "轴名:目标rad:整数百分比"
void Ros2Bridge::publishAnglePct(const std::string &axis, double angle_deg, int percent)
{
    double angle_rad = angle_deg * M_PI / 180.0;
    std::ostringstream oss;
    oss << axis << ":" << std::fixed << std::setprecision(6) << angle_rad
        << ":" << percent;   // 控制器要求纯整数百分比(5~100)，不能带小数点
    publishString(angle_pct_pub_, oss.str());
    emit logMessage(QString("[ANGLE pct=%1] %2:%3° (%4 rad)")
                        .arg(percent)
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

// ============================================================================
// 回原配置写入（方案C：直接修改 axes_config.yaml）
//   - 行级文本编辑：只动 operation_mode 那一行，注释与缩进全部保留
//   - 原子写 + .bak/.factory 备份 + 写入后自校验
// ============================================================================

bool Ros2Bridge::saveAxesHomingConfig(const std::map<std::string, bool> &homing_flags)
{
    if (axes_config_path_.empty()) {
        emit logMessage("[HOMING] 错误: 未记录 axes_config.yaml 路径，拒绝写入");
        return false;
    }
    if (homing_flags.empty()) {
        emit logMessage("[HOMING] 未指定任何轴，忽略本次写入");
        return false;
    }

    // 1) 读入原始文本（保留注释与换行风格）
    std::string raw;
    if (!readFileToString(axes_config_path_, raw)) {
        emit logMessage(QString("[HOMING] 无法读取配置文件: %1").arg(axes_config_path_.c_str()));
        return false;
    }
    const bool trailingNewline = (!raw.empty() && raw.back() == '\n');
    std::vector<std::string> lines = splitLines(raw);

    // 2) 定位每个轴条目（以 "- " 开头的列表项）
    struct AxisItem {
        size_t begin       = 0;
        size_t end         = 0;
        size_t nameLine    = 0;
        size_t fieldIndent = 0;
        std::string name;
        bool hasName = false;
    };
    std::vector<AxisItem> items;
    for (size_t i = 0; i < lines.size(); ++i) {
        const std::string t = stripCr(lines[i]);
        const size_t ind = leadingSpaces(t);
        if (ind + 1 < t.size() && t[ind] == '-' && t[ind + 1] == ' ') {
            AxisItem item;
            item.begin = i;
            items.push_back(item);
        }
        if (!items.empty()) {
            std::string nm;
            if (parseNameLine(lines[i], nm)) {
                items.back().name        = nm;
                items.back().hasName     = true;
                items.back().nameLine    = i;
                items.back().fieldIndent = leadingSpaces(stripCr(lines[i]));
            }
        }
    }
    for (size_t k = 0; k + 1 < items.size(); ++k) items[k].end = items[k + 1].begin;
    if (!items.empty()) items.back().end = lines.size();

    // 3) 生成编辑计划：只处理被点名的轴，其余字节完全不动
    struct Edit { size_t index; bool insert; std::string text; };
    std::vector<Edit> edits;
    QStringList touched;

    for (const auto &item : items) {
        if (!item.hasName) continue;
        const auto f = homing_flags.find(item.name);
        if (f == homing_flags.end()) continue;   // 未点名 → 保持文件原值

        size_t opLine = std::string::npos;
        size_t anchor = item.nameLine;            // 没有 operation_mode 时插到 calc_mode 之后
        for (size_t j = item.begin; j < item.end; ++j) {
            if (lineHasKeyAtIndent(lines[j], "operation_mode", item.fieldIndent)) {
                opLine = j;
                break;
            }
            if (lineHasKeyAtIndent(lines[j], "calc_mode", item.fieldIndent)) {
                anchor = j;
            }
        }

        const std::string newLine =
            std::string(item.fieldIndent, ' ') + "operation_mode: \"" +
            (f->second ? "homing" : "csp") + "\"";

        if (opLine != std::string::npos) {
            const bool cr = (!lines[opLine].empty() && lines[opLine].back() == '\r');
            edits.push_back({opLine, false, newLine + (cr ? "\r" : "")});
        } else {
            const bool cr = (!lines[anchor].empty() && lines[anchor].back() == '\r');
            edits.push_back({anchor, true, newLine + (cr ? "\r" : "")});
        }

        touched << QString("%1=%2").arg(QString::fromStdString(item.name))
                                      .arg(f->second ? "homing" : "csp");
    }

    if (edits.empty()) {
        emit logMessage("[HOMING] 未匹配到任何轴条目，配置未改动");
        return false;
    }

    // 从后往前应用，避免行号偏移
    std::sort(edits.begin(), edits.end(),
              [](const Edit &a, const Edit &b) { return a.index > b.index; });
    for (const auto &e : edits) {
        if (e.insert) {
            lines.insert(lines.begin() + static_cast<std::ptrdiff_t>(e.index + 1), e.text);
        } else {
            lines[e.index] = e.text;
        }
    }

    const std::string out = joinLines(lines, trailingNewline);

    // 4) 自校验：写出的内容必须能被控制器同款逻辑解析，否则放弃
    try {
        const YAML::Node check = YAML::Load(out);
        if (!check["servo_axes"] || !check["servo_axes"].IsSequence()) {
            emit logMessage("[HOMING] 自校验失败: servo_axes 缺失或不是数组，已放弃写入");
            return false;
        }
        const int newCount = static_cast<int>(check["servo_axes"].size());
        if (newCount != num_axes_) {
            emit logMessage(QString("[HOMING] 自校验失败: 轴数量由 %1 变为 %2，已放弃写入")
                                .arg(num_axes_).arg(newCount));
            return false;
        }
    } catch (const YAML::Exception &e) {
        emit logMessage(QString("[HOMING] 自校验失败(YAML 解析异常): %1，已放弃写入").arg(e.what()));
        return false;
    }

    // 5) 备份：.factory 只在首次写入前生成一次；.bak 每次覆盖
    const std::string factory = axes_config_path_ + ".factory";
    const std::string bak     = axes_config_path_ + ".bak";
    if (!fileExists(factory) && !copyFileTo(axes_config_path_, factory)) {
        emit logMessage("[HOMING] 生成出厂快照 .factory 失败，已放弃写入");
        return false;
    }
    if (!copyFileTo(axes_config_path_, bak)) {
        emit logMessage("[HOMING] 备份 .bak 失败，已放弃写入");
        return false;
    }

    // 6) 原子写入
    if (!atomicWriteFile(axes_config_path_, out)) {
        emit logMessage("[HOMING] 原子写入失败（临时文件/rename 出错）");
        return false;
    }

    // 7) 同步内存中的回原标记
    for (size_t i = 0; i < axis_names_.size(); ++i) {
        const auto f = homing_flags.find(axis_names_[i]);
        if (f != homing_flags.end()) axis_homing_values_[i] = f->second;
    }

    emit logMessage(QString("[HOMING] 已写入 %1 : %2")
                        .arg(QString::fromStdString(axes_config_path_), touched.join(", ")));
    emit logMessage("[HOMING] 注意: 需长按复位按钮 3 秒，控制器才会重新读取并使配置生效");
    return true;
}

bool Ros2Bridge::restoreAxesHomingFactory()
{
    if (axes_config_path_.empty()) {
        emit logMessage("[HOMING] 错误: 未记录 axes_config.yaml 路径");
        return false;
    }

    const std::string factory = axes_config_path_ + ".factory";
    std::string data;
    if (!readFileToString(factory, data)) {
        emit logMessage(QString("[HOMING] 未找到出厂快照 %1，无法恢复").arg(factory.c_str()));
        return false;
    }

    try {
        const YAML::Node check = YAML::Load(data);
        if (!check["servo_axes"] || !check["servo_axes"].IsSequence()) {
            emit logMessage("[HOMING] 出厂快照内容不合法，已放弃");
            return false;
        }
    } catch (const YAML::Exception &e) {
        emit logMessage(QString("[HOMING] 出厂快照解析失败: %1").arg(e.what()));
        return false;
    }

    copyFileTo(axes_config_path_, axes_config_path_ + ".bak");
    if (!atomicWriteFile(axes_config_path_, data)) {
        emit logMessage("[HOMING] 恢复出厂配置写入失败");
        return false;
    }
    emit logMessage("[HOMING] 已恢复出厂回原配置");
    return reloadAxesConfig();
}

bool Ros2Bridge::reloadAxesConfig()
{
    if (axes_config_path_.empty()) return false;
    return loadAxesFromYaml(axes_config_path_);
}
