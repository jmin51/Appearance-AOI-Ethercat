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
/// 提供 8 个发布器（含速度百分比指令），通过 Qt 信号通知 UI
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
    /// 第 idx 个轴当前的回原标记 (来自 YAML operation_mode, 写入后同步更新)
    bool getAxisHomingValue(int idx) const { return axis_homing_values_[idx]; }
    /// 第 idx 个轴是否列入回原白名单 (按 brand 判断, 供 UI 能力提示)
    bool isAxisHomingCapable(int idx) const { return axis_homing_capable_[idx]; }
    /// axes_config.yaml 的完整路径 (构造时记录)
    const std::string &getAxesConfigPath() const { return axes_config_path_; }

    // ————— 回原配置写入 (方案C: 直接改 axes_config.yaml) —————
    /// 将指定轴的回原标记写回 axes_config.yaml
    /// - 行级文本编辑, 保留原有注释与缩进
    /// - 写入前备份 .bak; 首次写入额外生成 .factory 出厂快照
    /// - 写入后自校验, 失败则不落盘
    /// @param homing_flags 轴名 -> 是否需要回原; 未包含的轴保持文件原值
    /// @return 是否写入成功
    bool saveAxesHomingConfig(const std::map<std::string, bool> &homing_flags);

    /// 用 .factory 出厂快照覆盖 axes_config.yaml, 并重新加载
    bool restoreAxesHomingFactory();

    /// 重新从 axes_config.yaml 加载轴配置 (供 UI 刷新回原标记)
    bool reloadAxesConfig();

    // ————— IO 配置访问接口 (从 io_config.yaml 动态加载) —————
    /// 获取DI输入信号列表 (M500~M547)
    const std::vector<IoSignalInfo> &getDiSignals() const { return di_signals_; }
    /// 获取DO输出信号列表 (M800~M847)
    const std::vector<IoSignalInfo> &getDoSignals() const { return do_signals_; }

public slots:
    // — 控制命令 —
    void publishControlCommand(const std::string &command);
    // — 点动命令 —
    void publishJogCommand(const std::string &axis, const std::string &direction);
    void publishJogStop(const std::string &axis);
    void publishJogStopAll();
    // — 位移命令（使用 YAML auto_speed 默认速度）—
    void publishDisplacement(const std::string &axis, double position_mm);
    // — 角度命令（使用 YAML auto_speed 默认速度）—
    void publishAngle(const std::string &axis, double angle_deg);
    // — 位移命令（带速度百分比，percent 取值 5~100 的整数）—
    void publishDisplacementPct(const std::string &axis, double position_mm, int percent);
    // — 角度命令（带速度百分比，percent 取值 5~100 的整数）—
    void publishAnglePct(const std::string &axis, double angle_deg, int percent);
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
    // 8 个发布器
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr jog_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr displacement_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr angle_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr io_control_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr virtual_io_pub_;
    // 速度百分比指令（走 /displacement_command_pct、/angle_command_pct）
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr displacement_pct_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr angle_pct_pub_;

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
    std::vector<bool>        axis_homing_values_;  // 当前 YAML 中的回原标记（初始值）
    std::vector<bool>        axis_homing_capable_; // 是否列入回原白名单（按 brand）
    std::string              axes_config_path_;    // axes_config.yaml 完整路径

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
