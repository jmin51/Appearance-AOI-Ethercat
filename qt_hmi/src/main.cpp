#include <QApplication>
#include <rclcpp/rclcpp.hpp>

#include "main_window.hpp"
#include "ros2_bridge.hpp"

int main(int argc, char *argv[])
{
    // 1. 先初始化 ROS2
    rclcpp::init(argc, argv);

    // 注册 std::string 和 std::vector 为 Qt 元类型 (必须在 connect 之前)
    qRegisterMetaType<std::string>("std::string");
    qRegisterMetaType<std::vector<std::string>>("std::vector<std::string>");
    qRegisterMetaType<std::vector<double>>("std::vector<double>");

    // 2. 创建 Qt 应用 (使用不同的 argc/argv 以避免 Qt 解析 ROS 参数)
    int qt_argc = 1;
    char qt_arg0[] = "qt_hmi";
    char *qt_argv[] = {qt_arg0, nullptr};
    QApplication app(qt_argc, qt_argv);

    // 3. 创建 ROS2 桥接 (含 rclcpp::Node)
    auto bridge = std::make_shared<Ros2Bridge>();

    // 4. 创建主窗口
    MainWindow window(bridge);
    window.setWindowTitle("EtherCAT Servo HMI");
    window.resize(1200, 800);
    window.show();

    // 5. 在独立线程中运行 rclcpp::spin
    std::thread spin_thread([bridge]() {
        rclcpp::spin(bridge);
        rclcpp::shutdown();
    });
    spin_thread.detach();

    // 6. 运行 Qt 事件循环
    int ret = app.exec();

    // 7. 确保 ROS2 关闭
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    if (spin_thread.joinable()) {
        spin_thread.join();
    }

    return ret;
}
