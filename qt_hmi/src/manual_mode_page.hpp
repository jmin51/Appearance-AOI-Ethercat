#pragma once

#include <QWidget>
#include <QSplitter>
#include <memory>

#include "ros2_bridge.hpp"
#include "jog_bar.hpp"
#include "io_panel.hpp"

/// 手动模式页面 — 左侧点动控制 + 右侧IO控制
/// 对应 control_commander.py 的 print_manual_menu()
class ManualModePage : public QWidget
{
    Q_OBJECT

public:
    explicit ManualModePage(std::shared_ptr<Ros2Bridge> bridge, QWidget *parent = nullptr);

public slots:
    /// 接收 Ros2Bridge 的 IO 状态更新
    void onIoStatusUpdated(const std::string &address, int value);
    /// 接收 /joint_states 实时位置, 分发给 JogBar
    void onJointStatesUpdated(const std::vector<std::string> &names,
                              const std::vector<double> &positions);

private:
    std::shared_ptr<Ros2Bridge> bridge_;

    JogBar *jog_bar_;
    IoPanel *io_panel_;
};
