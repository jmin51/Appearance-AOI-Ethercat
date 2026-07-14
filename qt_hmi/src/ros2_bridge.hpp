#pragma once

#include <QObject>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int8.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <map>
#include <string>
#include <thread>
#include <vector>

// 注册 std::vector 为 Qt 元类型, 解决跨线程 QueuedConnection 失败
Q_DECLARE_METATYPE(std::vector<std::string>)
Q_DECLARE_METATYPE(std::vector<double>)

/// IO 信号描述信息（从 io_config.yaml 加载）
struct IoSignalInfo {
    std::string name;       // 信号名 (如 "start_button_light")
    std::string address;    // Modbus地址 (如 "800")
    std::string desc;       // 中文描述 (如 "启动按钮灯")
    bool is_input;          // true=DI输入(只读), false=DO输出(可控)
};

/// ROS2 通信桥接类 — 同时继承 QObject 和 rclcpp::Node
/// 提供5个发布器，通过 Qt 信号通知 UI
/// 轴配置从 axes_config.yaml 动态加载
/// IO 配置从 io_config.yaml 动态加载
class Ros2Bridge : public QObject, public rclcpp::Node
{
    Q_OBJECT

public:
    explicit Ros2Bridge(QObject *parent = nullptr);

    // ————— 轴配置访问接口 (从 YAML 动态加载) —————
    /// 获取轴总数
    int getNumAxes() const { return num_axes_; }
    /// 获取第 idx 个轴的 ROS2 通信名 (如 "axis1")
    const std::string &getAxisName(int idx) const { return axis_names_[idx]; }
    /// 获取第 idx 个轴的显示名 (如 "X轴平移台")
    const std::string &getDisplayName(int idx) const { return axis_display_names_[idx]; }
    /// 获取第 idx 个轴的计算模式 ("DISPLACEMENT" / "ANGLE")
    const std::string &getCalcMode(int idx) const { return axis_calc_modes_[idx]; }
    /// 根据 ROS2 轴名查找索引, 未找到返回 -1
    int getAxisIndex(const std::string &name) const;

    // ————— IO 配置访问接口 (从 io_config.yaml 动态加载) —————
    /// 获取DI输入信号列表 (M500~M515)
    const std::vector<IoSignalInfo> &getDiSignals() const { return di_signals_; }
    /// 获取DO输出信号列表 (M800~M815)
    const std::vector<IoSignalInfo> &getDoSignals() const { return do_signals_; }

public slots:
    // — 控制命令 —
    void publishControlCommand(const std::string &command);
    // — 点动命令 —
    void publishJogCommand(const std::string &axis, const std::string &direction);
    void publishJogStop(const std::string &axis);
    void publishJogStopAll();
    // — 位移命令 —
    void publishDisplacement(const std::string &axis, double position_mm);
    // — 角度命令 —
    void publishAngle(const std::string &axis, double angle_deg);
    // — IO 控制 —
    void publishIoControl(const std::string &address, int value);
    // — 虚拟IO —
    void publishVirtualIo(const std::string &command);

signals:
    /// 日志消息信号 — MainWindow 接收并显示在日志面板
    void logMessage(const QString &msg);
    /// IO 状态更新信号 (address, value) — DO 信号状态变化
    void ioStatusUpdated(const std::string &address, int value);
    /// DI 输入状态更新信号 (address, value) — 来自 /io_status
    void diStatusUpdated(const std::string &address, int value);
    /// 系统状态更新信号 (来自 /system_status)
    void systemStatusChanged(const QString &status);
    /// 关节状态更新信号 (来自 /joint_states)
    void jointStatesUpdated(const std::vector<std::string> &names,
                            const std::vector<double> &positions);

private:
    // 5 个发布器
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr jog_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr displacement_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr angle_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr io_control_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr virtual_io_pub_;

    // 订阅器
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr system_status_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr io_status_sub_;

    // IO 状态跟踪
    std::map<std::string, int> di_status_;    // DI 输入状态 (M500~M515)
    std::map<std::string, int> do_status_;    // DO 输出状态 (M800~M815)

    // ————— 轴配置 (从 axes_config.yaml 动态加载) —————
    int num_axes_ = 0;
    std::vector<std::string> axis_names_;          // ROS2通信名 (如 "axis1")
    std::vector<std::string> axis_display_names_;  // 界面显示名 (如 "X轴平移台")
    std::vector<std::string> axis_calc_modes_;     // 计算模式 ("DISPLACEMENT"/"ANGLE")

    // ————— IO 配置 (从 io_config.yaml 动态加载) —————
    std::vector<IoSignalInfo> di_signals_;   // DI 输入信号列表
    std::vector<IoSignalInfo> do_signals_;   // DO 输出信号列表

    // 从 YAML 加载轴配置
    bool loadAxesFromYaml(const std::string &yamlPath);
    // 从 YAML 加载 IO 配置
    bool loadIoConfigFromYaml(const std::string &yamlPath);

    // 发布 String 消息的辅助方法
    void publishString(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub,
                       const std::string &data);
    // 系统状态回调
    void onSystemStatusReceived(const std_msgs::msg::String::SharedPtr msg);
    // 关节状态回调
    void onJointStatesReceived(const sensor_msgs::msg::JointState::SharedPtr msg);
    // IO 状态回调 (来自 /io_status)
    void onIoStatusReceived(const std_msgs::msg::String::SharedPtr msg);
};
