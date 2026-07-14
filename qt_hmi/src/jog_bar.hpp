#pragma once

#include <QWidget>
#include <QGridLayout>
#include <QPushButton>
#include <QLabel>
#include <vector>
#include <string>
#include <memory>

#include "ros2_bridge.hpp"

/// 点动控制条 — 8轴正/反转按钮 + 全停按钮 + 实时位置显示
/// 按下发送 forward/reverse，松开发送 stop (松手即停)
class JogBar : public QWidget
{
    Q_OBJECT

public:
    explicit JogBar(std::shared_ptr<Ros2Bridge> bridge, QWidget *parent = nullptr);
    /// 更新指定轴的实时位置显示 (来自 /joint_states)
    void updatePosition(int axis_idx, double value);

private slots:
    void onForwardPressed(int axis_idx);
    void onForwardReleased(int axis_idx);
    void onReversePressed(int axis_idx);
    void onReverseReleased(int axis_idx);
    void onStopAll();

private:
    std::shared_ptr<Ros2Bridge> bridge_;

    struct AxisRow {
        QLabel *label;
        QLabel *display_label;      // YAML display_name 显示
        QPushButton *forward_btn;
        QPushButton *reverse_btn;
        QLabel *position_label;     // 实时位置显示
    };

    std::vector<AxisRow> axis_rows_;
    QPushButton *stop_all_btn_;
};
