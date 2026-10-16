#include "axis_panel.hpp"

#include <QGroupBox>
#include <QLabel>
#include <QMessageBox>
#include <QVariant>
#include <cmath>

AxisPanel::AxisPanel(const std::string &axis_name,
                     const std::string &calc_mode,
                     const std::string &display_name,
                     std::shared_ptr<Ros2Bridge> bridge,
                     QWidget *parent)
    : QWidget(parent), axis_name_(axis_name), display_name_(display_name),
      calc_mode_(calc_mode), bridge_(bridge)
{
    auto *group = new QGroupBox(
        QString("%1 [%2]").arg(display_name.c_str()).arg(calc_mode.c_str()), this);
    auto *layout = new QVBoxLayout(group);

    // 位置显示
    pos_label_ = new QLabel("位置: 0.000 mm", this);
    pos_label_->setStyleSheet("QLabel { font-weight: bold; font-size: 13px; }");
    layout->addWidget(pos_label_);

    if (calc_mode_ == "DISPLACEMENT") {
        // 位移轴: 6 个按钮 (与Python的disp_origin/disp_preset/disp_forward/disp_reverse/disp_input/angle_input 对应)

        // 步长调节
        auto *step_layout = new QHBoxLayout();
        step_layout->addWidget(new QLabel("步长(mm):", this));
        step_spin_ = new QDoubleSpinBox(this);
        step_spin_->setRange(1.0, 100.0);
        step_spin_->setValue(10.0);
        step_spin_->setSingleStep(1.0);
        step_spin_->setDecimals(1);
        step_layout->addWidget(step_spin_);
        layout->addLayout(step_layout);

        // 速度百分比选择 ("默认" = 使用 YAML auto_speed；其余档走 *_pct 话题)
        auto *speed_layout = new QHBoxLayout();
        speed_layout->addWidget(new QLabel("速度(%):", this));
        initSpeedCombo();
        speed_layout->addWidget(speed_pct_combo_);
        speed_layout->addStretch();
        layout->addLayout(speed_layout);

        // 按钮网格 3×2
        auto *btn_grid = new QGridLayout();

        QPushButton *btn_origin = new QPushButton("回原点 (0)", this);
        QPushButton *btn_preset = new QPushButton("预设 (10mm)", this);
        QPushButton *btn_fwd    = new QPushButton("+步进", this);
        QPushButton *btn_rev    = new QPushButton("-步进", this);
        QPushButton *btn_disp   = new QPushButton("位移输入(mm)", this);
        QPushButton *btn_angle  = new QPushButton("角度输入(°)", this);

        btn_origin->setStyleSheet("QPushButton { background-color: #607D8B; color: white; }");
        btn_preset->setStyleSheet("QPushButton { background-color: #795548; color: white; }");
        btn_fwd->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; }");
        btn_rev->setStyleSheet("QPushButton { background-color: #F44336; color: white; }");
        btn_disp->setStyleSheet("QPushButton { background-color: #2196F3; color: white; }");
        btn_angle->setStyleSheet("QPushButton { background-color: #9C27B0; color: white; }");

        connect(btn_origin, &QPushButton::clicked, this, &AxisPanel::onOrigin);
        connect(btn_preset, &QPushButton::clicked, this, &AxisPanel::onPreset);
        connect(btn_fwd,    &QPushButton::clicked, this, &AxisPanel::onForwardStep);
        connect(btn_rev,    &QPushButton::clicked, this, &AxisPanel::onReverseStep);
        connect(btn_disp,   &QPushButton::clicked, this, &AxisPanel::onDispInput);
        connect(btn_angle,  &QPushButton::clicked, this, &AxisPanel::onAngleInput);

        btn_grid->addWidget(btn_origin, 0, 0);
        btn_grid->addWidget(btn_preset, 0, 1);
        btn_grid->addWidget(btn_fwd,    1, 0);
        btn_grid->addWidget(btn_rev,    1, 1);
        btn_grid->addWidget(btn_disp,   2, 0);
        btn_grid->addWidget(btn_angle,  2, 1);

        layout->addLayout(btn_grid);
    } else {
        // 角度轴: 2 个按钮 (angle_input + disp_input)
        step_spin_ = nullptr;

        // 速度百分比选择 ("默认" = 使用 YAML auto_speed；其余档走 *_pct 话题)
        auto *speed_layout = new QHBoxLayout();
        speed_layout->addWidget(new QLabel("速度(%):", this));
        initSpeedCombo();
        speed_layout->addWidget(speed_pct_combo_);
        speed_layout->addStretch();
        layout->addLayout(speed_layout);

        auto *btn_layout = new QHBoxLayout();

        QPushButton *btn_angle = new QPushButton("角度输入(°)", this);
        QPushButton *btn_disp  = new QPushButton("位移输入(mm)", this);

        btn_angle->setStyleSheet("QPushButton { background-color: #9C27B0; color: white; min-height: 35px; }");
        btn_disp->setStyleSheet("QPushButton { background-color: #2196F3; color: white; min-height: 35px; }");

        connect(btn_angle, &QPushButton::clicked, this, &AxisPanel::onAngleInputOnly);
        connect(btn_disp,  &QPushButton::clicked, this, &AxisPanel::onDispInputOnly);

        btn_layout->addWidget(btn_angle);
        btn_layout->addWidget(btn_disp);

        layout->addLayout(btn_layout);
    }

    auto *outer = new QVBoxLayout(this);
    outer->addWidget(group);
}

void AxisPanel::updatePositionDisplay(double pos_mm)
{
    position_mm_ = pos_mm;
    pos_label_->setText(QString("位置: %1 mm").arg(pos_mm, 0, 'f', 3));
}

void AxisPanel::updateJointPosition(double value)
{
    if (calc_mode_ == "DISPLACEMENT") {
        position_mm_ = value;
        pos_label_->setText(QString("位置: %1 mm").arg(value, 0, 'f', 3));
    } else {
        // ANGLE 轴: /joint_states 发 rad, 转 deg 显示
        position_deg_ = value * 180.0 / M_PI;
        pos_label_->setText(QString("位置: %1°").arg(position_deg_, 0, 'f', 2));
    }
}

// ————— DISPLACEMENT 轴操作 —————

void AxisPanel::onOrigin()
{
    position_mm_ = 0.0;
    sendDisplacement(0.0);
    updatePositionDisplay(0.0);
}

void AxisPanel::onPreset()
{
    position_mm_ = 10.0;
    sendDisplacement(10.0);
    updatePositionDisplay(10.0);
}

void AxisPanel::onForwardStep()
{
    double step = step_spin_->value();
    position_mm_ += step;
    sendDisplacement(position_mm_);
    updatePositionDisplay(position_mm_);
}

void AxisPanel::onReverseStep()
{
    double step = step_spin_->value();
    position_mm_ -= step;
    sendDisplacement(position_mm_);
    updatePositionDisplay(position_mm_);
}

void AxisPanel::onDispInput()
{
    bool ok = false;
    double val = QInputDialog::getDouble(this,
        QString("%1 位移输入").arg(display_name_.c_str()),
        "请输入位移值(mm):",
        position_mm_, -10000.0, 10000.0, 3, &ok);
    if (ok) {
        position_mm_ = val;
        sendDisplacement(val);
        updatePositionDisplay(val);
    }
}

void AxisPanel::onAngleInput()
{
    bool ok = false;
    double deg = QInputDialog::getDouble(this,
        QString("%1 角度输入").arg(display_name_.c_str()),
        "请输入角度值(°):",
        position_deg_, -360.0, 360.0, 2, &ok);
    if (ok) {
        position_deg_ = deg;
        sendAngle(deg);
    }
}

// ————— ANGLE 轴操作 —————

void AxisPanel::onAngleInputOnly()
{
    bool ok = false;
    double deg = QInputDialog::getDouble(this,
        QString("%1 角度输入").arg(display_name_.c_str()),
        "请输入角度值(°):",
        position_deg_, -360.0, 360.0, 2, &ok);
    if (ok) {
        position_deg_ = deg;
        sendAngle(deg);
    }
}

void AxisPanel::onDispInputOnly()
{
    bool ok = false;
    double val = QInputDialog::getDouble(this,
        QString("%1 位移输入").arg(display_name_.c_str()),
        "请输入位移值(mm):",
        position_mm_, -10000.0, 10000.0, 3, &ok);
    if (ok) {
        position_mm_ = val;
        sendDisplacement(val);
        updatePositionDisplay(val);
    }
}

// ————— 速度百分比辅助 —————
// 百分比相对该轴的 max_jog_speed，取值 5~100 的整数。
// 控制器会拒收 <5、>100 以及 "70.0" 这类非整数写法，因此这里只提供整数档位，
// 并且不提供 0："默认"档走不带百分比的原话题，而不是发送 0。
void AxisPanel::initSpeedCombo()
{
    speed_pct_combo_ = new QComboBox(this);

    // 索引 0 = "默认"：userData 为无效 QVariant，selectedPercent() 据此返回 -1
    speed_pct_combo_->addItem("默认", QVariant());

    static const int kPercents[] = {5, 10, 20, 30, 50, 70, 80, 100};
    for (int pct : kPercents) {
        speed_pct_combo_->addItem(QString::number(pct) + "%", pct);
    }

    speed_pct_combo_->setCurrentIndex(0);   // 默认档，保持原有操作手感
    speed_pct_combo_->setToolTip(
        "移动速度百分比（相对该轴 max_jog_speed，范围 5~100）\n"
        "\"默认\" = 使用 axes_config.yaml 中该轴的 auto_speed");
}

int AxisPanel::selectedPercent() const
{
    if (!speed_pct_combo_) {
        return -1;   // 无控件（理论上不会发生）→ 退回默认速度
    }
    bool ok = false;
    const int pct = speed_pct_combo_->currentData().toInt(&ok);
    return ok ? pct : -1;   // "默认"档 userData 无效 → ok=false → -1
}

void AxisPanel::sendDisplacement(double position_mm)
{
    const int pct = selectedPercent();
    if (pct < 0) {
        // 默认速度：控制器使用该轴 YAML 的 auto_speed
        bridge_->publishDisplacement(axis_name_, position_mm);
    } else {
        // 指定百分比：走 /displacement_command_pct
        bridge_->publishDisplacementPct(axis_name_, position_mm, pct);
    }
}

void AxisPanel::sendAngle(double angle_deg)
{
    const int pct = selectedPercent();
    if (pct < 0) {
        bridge_->publishAngle(axis_name_, angle_deg);
    } else {
        bridge_->publishAnglePct(axis_name_, angle_deg, pct);
    }
}
