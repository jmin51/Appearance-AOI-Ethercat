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
    /// 虚拟IO按钮
    void onVirtualStart();
    void onVirtualEstop();

private:
    void setupVirtualPanel();
    void setupToolBar();
    void setupCentralWidget();
    void setupStatusBar();
    /// 根据当前 HmiMode 同步工具栏按钮启用/禁用状态
    void syncToolbarState();
    /// 设置 HMI 模式并同步工具栏 + 状态栏
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
    QPushButton *btn_virtual_estop_ = nullptr;

    // 虚拟IO面板容器
    QFrame *virtual_panel_ = nullptr;

    // 当前 HMI 模式
    HmiMode hmi_mode_ = HmiMode::INIT;
};
