#pragma once

#include <QWidget>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QInputDialog>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <string>
#include <map>
#include <memory>

#include "ros2_bridge.hpp"

/// 单轴位置控制面板 — 根据轴的 calc_mode 提供不同按钮组合
/// DISPLACEMENT 轴 (axes_config.yaml 中 axis1~axis5): 回原点/预设/正步进/负步进/位移输入/角度输入
/// ANGLE 轴 (axes_config.yaml 中 axis6): 角度输入/位移输入
/// 两种轴均支持"速度百分比"选择：默认档走 auto_speed，其余档走 *_pct 话题
class AxisPanel : public QWidget
{
    Q_OBJECT

public:
    explicit AxisPanel(const std::string &axis_name,
                       const std::string &calc_mode,
                       const std::string &display_name,
                       std::shared_ptr<Ros2Bridge> bridge,
                       QWidget *parent = nullptr);

    /// 获取轴名
    const std::string &axisName() const { return axis_name_; }
    /// 更新轴位置显示 (手动操作后本地更新)
    void updatePositionDisplay(double pos_mm);
    /// 更新轴位置显示 (/joint_states 实时反馈)
    void updateJointPosition(double value);

private slots:
    // DISPLACEMENT 轴按钮
    void onOrigin();
    void onPreset();
    void onForwardStep();
    void onReverseStep();
    void onDispInput();
    void onAngleInput();
    // ANGLE 轴按钮
    void onAngleInputOnly();
    void onDispInputOnly();

private:
    std::string axis_name_;
    std::string display_name_;
    std::string calc_mode_;
    std::shared_ptr<Ros2Bridge> bridge_;

    double position_mm_ = 0.0;     // 当前位移位置
    double position_deg_ = 0.0;    // 当前角度位置
    double step_mm_ = 10.0;        // 默认步长10mm (与Python一致)

    QLabel *pos_label_;             // 位置显示标签
    QDoubleSpinBox *step_spin_;     // 步长调节 (仅DISPLACEMENT轴显示)
    QComboBox *speed_pct_combo_ = nullptr;  // 速度百分比选择 ("默认" / 5~100)

    // ————— 内部辅助 —————
    /// 创建并填充速度百分比下拉框 (创建后需由调用方加入布局)
    void initSpeedCombo();
    /// 当前选择的速度百分比; 返回 -1 表示"默认速度"(使用 YAML 的 auto_speed)
    int selectedPercent() const;
    /// 发送位移指令: 按当前百分比选择 /displacement_command 或 /displacement_command_pct
    void sendDisplacement(double position_mm);
    /// 发送角度指令(单位:度): 按当前百分比选择 /angle_command 或 /angle_command_pct
    void sendAngle(double angle_deg);
};
