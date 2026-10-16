#pragma once

#include <QMainWindow>
#include <QAction>
#include <QTabWidget>
#include <QTextEdit>
#include <QToolBar>
#include <QStatusBar>
#include <QLabel>
#include <QFrame>
#include <QHBoxLayout>
#include <QPushButton>
#include <QComboBox>
#include <vector>
#include <memory>

#include "ros2_bridge.hpp"
#include "manual_mode_page.hpp"
#include "auto_mode_page.hpp"

/// HMI 模式状态机 — 跟踪用户在界面层面的操作阶段
enum class HmiMode {
    INIT = 0,    // 轴初始化中，不可操作
    READY,       // 轴就绪，可选择手/自动模式
    MANUAL,      // 手动模式运行中
    AUTO,        // 自动模式运行中
    STOPPED,     // 已停机，可重新选择模式
    FAULT        // 故障状态，需清除故障
};

/// 主窗口 — 工具栏 + 标签页 + 日志面板 + 状态栏
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(std::shared_ptr<Ros2Bridge> bridge, QWidget *parent = nullptr);

private slots:
    void onEnterManual();
    void onEnterAuto();
    void onStopAll();
    void onClearFault();
    void onResetAxes();
    void onResetAllIo();
    void onLogMessage(const QString &msg);
    /// 接收控制器的系统状态反馈
    void onSystemStatusReceived(const QString &status);
    /// 虚拟IO按钮（按下/释放事件对，与实体按钮按压语义一致）
    void onVirtualStartPressed();
    void onVirtualStartReleased();
    void onVirtualResetPressed();
    void onVirtualResetReleased();
    void onVirtualPausePressed();
    void onVirtualPauseReleased();
    /// 虚拟急停（点击切换锁存：再点一次解除）
    void onVirtualEstop();
    /// 回原设置：应用 / 重新载入 / 恢复出厂
    void onApplyHomingConfig();
    void onReloadHomingConfig();
    void onRestoreFactoryHoming();

private:
    void setupVirtualPanel();
    void setupToolBar();
    void setupCentralWidget();
    void setupStatusBar();
    /// 构建"回原设置"面板（每轴独立配置）
    void setupHomingPanel();
    /// 用 Ros2Bridge 中的回原标记刷新下拉框
    void refreshHomingPanelFromBridge();
    /// 根据控制器回报的状态同步面板可编辑性与提示文案
    void syncHomingPanelState();
    /// 是否允许修改回原配置（必须以控制器回报状态为准，不能用乐观值）
    bool canEditHoming() const;
    /// 根据当前 HmiMode 同步工具栏按钮启用/禁用状态
    void syncToolbarState();
    /// 暂停态渲染：状态栏显示"暂停中/已预选"，并按"只能预选一次"刷新按钮门控
    /// （只在控制器上报 "|PAUSED" 期间调用，不修改 hmi_mode_）
    void applyPausedUiState();
    /// 按模式渲染界面（状态栏 + 页签 + 工具栏门控）。**不做"模式未变则早退"**，
    /// 供 setHmiMode 与"解除暂停后强制重刷"两处调用
    void renderModeUi(HmiMode mode);
    /// 设置 HMI 模式并同步工具栏 + 状态栏（模式未变时早退）
    void setHmiMode(HmiMode mode);

    std::shared_ptr<Ros2Bridge> bridge_;

    // UI 元素
    QToolBar *toolbar_;
    QTabWidget *tab_widget_;
    QTextEdit *log_panel_;
    QLabel *mode_label_;
    QLabel *status_label_;

    // 页面
    ManualModePage *manual_page_;
    AutoModePage *auto_page_;

    // 工具栏按钮 (保存指针以便动态启用/禁用)
    QAction *act_manual_ = nullptr;
    QAction *act_auto_   = nullptr;
    QAction *act_stop_   = nullptr;
    QAction *act_fault_  = nullptr;
    QAction *act_reset_  = nullptr;
    QAction *act_io_     = nullptr;

    // 虚拟IO面板按钮
    QPushButton *btn_virtual_start_ = nullptr;
    QPushButton *btn_virtual_reset_ = nullptr;
    QPushButton *btn_virtual_pause_ = nullptr;
    QPushButton *btn_virtual_estop_ = nullptr;

    // 虚拟急停锁存状态（由控制器 /system_status 的 "|ESTOP" 驱动，用于按钮显示）
    bool virtual_estop_latched_ = false;
    /// 同步虚拟急停按钮的显示（锁存/未锁存）
    void syncVirtualEstopButton();

    /// 同步虚拟暂停按钮的显示（暂停中/运行中）
    /// 状态源唯一：控制器 /system_status 上报的 "|PAUSED"（controller_paused_），不再用本地点击计数
    void syncVirtualPauseButton();

    // 虚拟IO面板容器
    QFrame *virtual_panel_ = nullptr;

    // ————— 回原设置面板 —————
    QFrame *homing_panel_ = nullptr;
    QLabel  *homing_hint_label_ = nullptr;
    std::vector<QComboBox *> homing_combos_;   // 与轴索引一一对应
    QPushButton *btn_homing_apply_   = nullptr;
    QPushButton *btn_homing_reload_  = nullptr;
    QPushButton *btn_homing_factory_ = nullptr;
    bool homing_pending_apply_ = false;        // 已保存但尚未生效

    // 当前 HMI 模式（含乐观更新）
    HmiMode hmi_mode_ = HmiMode::INIT;
    // 控制器最近一次真实回报的状态 —— 回原设置门控以此为准，避免乐观更新导致误放行
    HmiMode controller_reported_mode_ = HmiMode::INIT;

    // 控制器上报的"暂停中"标记（/system_status 的 "|PAUSED" 后缀）
    bool controller_paused_ = false;
    // 本次暂停中 HMI 本地"已预选"的模式：0=未选、1=手动、2=自动
    // （仅用于界面"只能预选一次"的门控；真正的预选由控制器 g_pending_mode 记录）
    int  pending_mode_local_ = 0;
};
