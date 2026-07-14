#include "jog_bar.hpp"

#include <QFrame>

JogBar::JogBar(std::shared_ptr<Ros2Bridge> bridge, QWidget *parent)
    : QWidget(parent), bridge_(bridge)
{
    int num_axes = bridge_->getNumAxes();

    // 用 QFrame 替代 QGroupBox — 消除 Linux Fusion 风格下 GroupBox 标题间距不可控的问题
    auto *frame = new QFrame(this);
    frame->setFrameStyle(QFrame::StyledPanel | QFrame::Plain);
    frame->setStyleSheet("QFrame { border: 1px solid #aaa; border-radius: 3px; }");

    auto *vbox = new QVBoxLayout(frame);
    vbox->setContentsMargins(4, 2, 4, 4);
    vbox->setSpacing(2);

    auto *title = new QLabel("点动控制 (按下运行, 松手即停)");
    title->setStyleSheet("QLabel { font-weight: bold; font-size: 17px; min-height: 32px; }");
    vbox->addWidget(title);

    auto *grid = new QGridLayout();
    grid->setSpacing(4);
    grid->setContentsMargins(0, 2, 0, 0);
    grid->setVerticalSpacing(3);

    // 标题行: 轴号 | 显示名 | 正转 | 反转 | 当前位置
    grid->addWidget(new QLabel("轴号"), 0, 0);
    grid->addWidget(new QLabel("显示名"), 0, 1);
    grid->addWidget(new QLabel("正转 ↗"), 0, 2);
    grid->addWidget(new QLabel("反转 ↘"), 0, 3);
    grid->addWidget(new QLabel("当前位置"), 0, 4);

    // 动态轴按钮行
    for (int i = 0; i < num_axes; ++i) {
        AxisRow row;
        std::string axis_name     = bridge_->getAxisName(i);
        std::string display_name  = bridge_->getDisplayName(i);

        row.label = new QLabel(QString("轴%1").arg(i + 1), this);
        row.display_label = new QLabel(QString::fromStdString(display_name), this);
        row.display_label->setStyleSheet("QLabel { font-weight: bold; color: #1565C0; }");
        row.forward_btn = new QPushButton(QString("正转 ↗"), this);
        row.reverse_btn = new QPushButton(QString("反转 ↘"), this);
        row.position_label = new QLabel("---", this);
        row.position_label->setAlignment(Qt::AlignCenter);
        row.position_label->setStyleSheet("QLabel { font-weight: bold; color: #333; }");

        // 设置按钮样式：按下变红色，提示危险操作
        row.forward_btn->setStyleSheet(
            "QPushButton { background-color: #4CAF50; color: white; font-weight: bold; min-height: 22px; }"
            "QPushButton:pressed { background-color: #E53935; }");
        row.reverse_btn->setStyleSheet(
            "QPushButton { background-color: #2196F3; color: white; font-weight: bold; min-height: 22px; }"
            "QPushButton:pressed { background-color: #E53935; }");

        // 按下/松开信号 → 点动安全 (松手即停)
        connect(row.forward_btn, &QPushButton::pressed, this, [this, i]() { onForwardPressed(i); });
        connect(row.forward_btn, &QPushButton::released, this, [this, i]() { onForwardReleased(i); });
        connect(row.reverse_btn, &QPushButton::pressed, this, [this, i]() { onReversePressed(i); });
        connect(row.reverse_btn, &QPushButton::released, this, [this, i]() { onReverseReleased(i); });

        grid->addWidget(row.label,          i + 1, 0);
        grid->addWidget(row.display_label,  i + 1, 1);
        grid->addWidget(row.forward_btn,    i + 1, 2);
        grid->addWidget(row.reverse_btn,    i + 1, 3);
        grid->addWidget(row.position_label, i + 1, 4);

        axis_rows_.push_back(row);
    }

    // 全停按钮
    stop_all_btn_ = new QPushButton("停止所有轴", this);
    stop_all_btn_->setStyleSheet(
        "QPushButton { background-color: #E53935; color: white; font-weight: bold; font-size: 13px; min-height: 28px; }");
    connect(stop_all_btn_, &QPushButton::clicked, this, &JogBar::onStopAll);
    grid->addWidget(stop_all_btn_, num_axes + 1, 0, 1, 5);

    grid->setRowStretch(grid->rowCount(), 1);

    vbox->addLayout(grid);

    auto *outer = new QVBoxLayout(this);
    outer->setContentsMargins(0, 0, 0, 0);
    outer->addWidget(frame);
}

void JogBar::onForwardPressed(int axis_idx)
{
    bridge_->publishJogCommand(bridge_->getAxisName(axis_idx), "forward");
}

void JogBar::onForwardReleased(int axis_idx)
{
    bridge_->publishJogStop(bridge_->getAxisName(axis_idx));
}

void JogBar::onReversePressed(int axis_idx)
{
    bridge_->publishJogCommand(bridge_->getAxisName(axis_idx), "reverse");
}

void JogBar::onReverseReleased(int axis_idx)
{
    bridge_->publishJogStop(bridge_->getAxisName(axis_idx));
}

void JogBar::onStopAll()
{
    bridge_->publishJogStopAll();
}

void JogBar::updatePosition(int axis_idx, double value)
{
    if (axis_idx < 0 || axis_idx >= static_cast<int>(axis_rows_.size()))
        return;

    const std::string &calc_mode = bridge_->getCalcMode(axis_idx);

    if (calc_mode == "DISPLACEMENT") {
        axis_rows_[axis_idx].position_label->setText(
            QString("%1 mm").arg(value, 0, 'f', 3));
    } else {
        // ANGLE 轴: /joint_states 发 rad, 转 deg 显示
        double deg = value * 180.0 / M_PI;
        axis_rows_[axis_idx].position_label->setText(
            QString("%1°").arg(deg, 0, 'f', 2));
    }
}
