#pragma once

#include <QWidget>
#include <QScrollArea>
#include <QSplitter>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <vector>
#include <memory>

#include "ros2_bridge.hpp"
#include "io_panel.hpp"
#include "axis_panel.hpp"

/// 自动模式页面 — 轴位置控制(AxisPanel×8) + IO控制
/// 对应 control_commander.py 的 print_auto_menu()
class AutoModePage : public QWidget
{
    Q_OBJECT

public:
    explicit AutoModePage(std::shared_ptr<Ros2Bridge> bridge, QWidget *parent = nullptr);

public slots:
    /// 接收 Ros2Bridge 的 IO 状态更新
    void onIoStatusUpdated(const std::string &address, int value);
    /// 接收 /joint_states 实时位置, 分发给各 AxisPanel
    void onJointStatesUpdated(const std::vector<std::string> &names,
                              const std::vector<double> &positions);

private:
    std::shared_ptr<Ros2Bridge> bridge_;

    // 8 个轴面板 (根据 calc_mode 不同)
    std::vector<AxisPanel*> axis_panels_;
    IoPanel *io_panel_;
};
