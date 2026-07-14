#include "main_window.hpp"

#include <QAction>
#include <QDateTime>
#include <QSplitter>

MainWindow::MainWindow(std::shared_ptr<Ros2Bridge> bridge, QWidget *parent)
    : QMainWindow(parent), bridge_(bridge)
{
    setupToolBar();
    setupCentralWidget();
    setupStatusBar();

    // 连接 Ros2Bridge 的日志信号
    connect(bridge_.get(), &Ros2Bridge::logMessage,
            this, &MainWindow::onLogMessage);

    // 连接 IO 状态信号 → 两个页面都接收更新
    connect(bridge_.get(), &Ros2Bridge::ioStatusUpdated,
            manual_page_, &ManualModePage::onIoStatusUpdated);
    connect(bridge_.get(), &Ros2Bridge::ioStatusUpdated,
            auto_page_, &AutoModePage::onIoStatusUpdated);

    // 连接系统状态信号 → 同步 HMI 模式
    connect(bridge_.get(), &Ros2Bridge::systemStatusChanged,
            this, &MainWindow::onSystemStatusReceived);

    // 初始按钮状态
    syncToolbarState();

    onLogMessage("[HMI] MainWindow initialized");
}

void MainWindow::setupToolBar()
{
    toolbar_ = addToolBar("Main Actions");
    toolbar_->setMovable(false);
    toolbar_->setIconSize(QSize(32, 32));

    // 6 个快捷操作按钮 — 保存 QAction 指针以便动态启用/禁用
    act_manual_ = toolbar_->addAction("手动模式");
    act_auto_   = toolbar_->addAction("自动模式");
    toolbar_->addSeparator();
    act_stop_   = toolbar_->addAction("停机");
    act_fault_  = toolbar_->addAction("清除故障");
    act_reset_  = toolbar_->addAction("重置轴");
    toolbar_->addSeparator();
    act_io_     = toolbar_->addAction("IO全复位");

    connect(act_manual_, &QAction::triggered, this, &MainWindow::onEnterManual);
    connect(act_auto_,   &QAction::triggered, this, &MainWindow::onEnterAuto);
    connect(act_stop_,   &QAction::triggered, this, &MainWindow::onStopAll);
    connect(act_fault_,  &QAction::triggered, this, &MainWindow::onClearFault);
    connect(act_reset_,  &QAction::triggered, this, &MainWindow::onResetAxes);
    connect(act_io_,     &QAction::triggered, this, &MainWindow::onResetAllIo);
}

void MainWindow::setupCentralWidget()
{
    // 创建虚拟IO面板（放在标签页上方）
    virtual_panel_ = new QFrame(this);
    virtual_panel_->setFrameStyle(QFrame::StyledPanel | QFrame::Plain);
    virtual_panel_->setStyleSheet(
        "QFrame { background-color: #f5f5f5; border-bottom: 2px solid #ccc; }");

    auto *vLayout = new QHBoxLayout(virtual_panel_);
    vLayout->setContentsMargins(8, 4, 8, 4);

    auto *vLabel = new QLabel("虚拟按钮", virtual_panel_);
    vLabel->setStyleSheet("font-weight: bold; font-size: 14px;");
    vLayout->addWidget(vLabel);
    vLayout->addStretch();

    btn_virtual_start_ = new QPushButton("▶ 虚拟启动", virtual_panel_);
    btn_virtual_start_->setStyleSheet(
        "QPushButton { background-color: #4CAF50; color: white; font-weight: bold;"
        "min-height: 36px; min-width: 120px; font-size: 14px; border-radius: 4px; }"
        "QPushButton:hover { background-color: #45a049; }");
    vLayout->addWidget(btn_virtual_start_);

    btn_virtual_estop_ = new QPushButton("■ 虚拟急停", virtual_panel_);
    btn_virtual_estop_->setStyleSheet(
        "QPushButton { background-color: #E53935; color: white; font-weight: bold;"
        "min-height: 36px; min-width: 120px; font-size: 14px; border-radius: 4px; }"
        "QPushButton:hover { background-color: #c62828; }");
    vLayout->addWidget(btn_virtual_estop_);

    connect(btn_virtual_start_, &QPushButton::clicked, this, &MainWindow::onVirtualStart);
    connect(btn_virtual_estop_, &QPushButton::clicked, this, &MainWindow::onVirtualEstop);

    // 标签页
    tab_widget_ = new QTabWidget(this);
    manual_page_ = new ManualModePage(bridge_, this);
    auto_page_   = new AutoModePage(bridge_, this);
    tab_widget_->addTab(manual_page_, "手动模式");
    tab_widget_->addTab(auto_page_,   "自动模式");

    // 日志面板
    log_panel_ = new QTextEdit(this);
    log_panel_->setReadOnly(true);
    log_panel_->setMinimumHeight(0);
    log_panel_->setMaximumHeight(200);
    log_panel_->setStyleSheet(
        "QTextEdit { background-color: #1e1e1e; color: #d4d4d4; font-family: monospace; font-size: 12px; }");

    // 垂直分割：标签页 + 日志
    auto *splitter = new QSplitter(Qt::Vertical, this);
    splitter->addWidget(tab_widget_);
    splitter->addWidget(log_panel_);
    splitter->setStretchFactor(0, 4);
    splitter->setStretchFactor(1, 1);

    // 主布局：虚拟面板 + 分割器
    auto *mainWidget = new QWidget(this);
    auto *mainLayout = new QVBoxLayout(mainWidget);
    mainLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->setSpacing(0);
    mainLayout->addWidget(virtual_panel_);
    mainLayout->addWidget(splitter, 1);

    setCentralWidget(mainWidget);

    // 设置合理的窗口最小高度，避免子控件隐式最小高度过大导致无法缩小
    setMinimumHeight(480);
}

void MainWindow::setupStatusBar()
{
    mode_label_ = new QLabel("模式: 初始化中...", this);
    status_label_ = new QLabel("状态: 等待轴就绪", this);
    statusBar()->addWidget(mode_label_, 1);
    statusBar()->addPermanentWidget(status_label_);
}

// ————— HMI 模式管理 —————

void MainWindow::setHmiMode(HmiMode mode)
{
    if (hmi_mode_ == mode) return;
    hmi_mode_ = mode;

    // 更新状态栏
    switch (mode)
    {
    case HmiMode::INIT:
        mode_label_->setText("模式: 初始化中...");
        status_label_->setText("状态: 等待轴就绪");
        break;
    case HmiMode::READY:
        mode_label_->setText("模式: 就绪");
        status_label_->setText("状态: 请选择手/自动模式");
        break;
    case HmiMode::MANUAL:
        mode_label_->setText("模式: 手动运行中");
        status_label_->setText("状态: 点动控制可用");
        break;
    case HmiMode::AUTO:
        mode_label_->setText("模式: 自动运行中");
        status_label_->setText("状态: 自动序列执行中");
        break;
    case HmiMode::STOPPED:
        mode_label_->setText("模式: 已停机");
        status_label_->setText("状态: 可重新选择模式");
        break;
    case HmiMode::FAULT:
        mode_label_->setText("模式: 故障");
        status_label_->setText("状态: 请清除故障后重试");
        break;
    }

    syncToolbarState();
}

void MainWindow::syncToolbarState()
{
    if (!act_manual_ || !act_auto_ || !act_stop_ ||
        !act_fault_ || !act_reset_ || !act_io_)
        return;

    // 按钮启用规则表
    //             手动模式  自动模式  停机   清除故障  重置轴  IO全复位
    // INIT          ❌       ❌      ❌     ❌      ❌      ❌
    // READY         ✅       ✅      ❌     ❌      ❌      ❌
    // MANUAL        ❌       ❌      ✅     ❌      ❌      ✅
    // AUTO          ❌       ❌      ✅     ❌      ❌      ✅
    // STOPPED       ✅       ✅      ❌     ❌      ✅      ✅
    // FAULT         ❌       ❌      ✅     ✅      ✅      ❌

    switch (hmi_mode_)
    {
    case HmiMode::INIT:
        act_manual_->setEnabled(false);
        act_auto_->setEnabled(false);
        act_stop_->setEnabled(false);
        act_fault_->setEnabled(false);
        act_reset_->setEnabled(false);
        act_io_->setEnabled(false);
        break;

    case HmiMode::READY:
        act_manual_->setEnabled(true);
        act_auto_->setEnabled(true);
        act_stop_->setEnabled(false);
        act_fault_->setEnabled(false);
        act_reset_->setEnabled(false);
        act_io_->setEnabled(false);
        break;

    case HmiMode::MANUAL:
        act_manual_->setEnabled(false);
        act_auto_->setEnabled(false);
        act_stop_->setEnabled(true);
        act_fault_->setEnabled(false);
        act_reset_->setEnabled(false);
        act_io_->setEnabled(true);
        break;

    case HmiMode::AUTO:
        act_manual_->setEnabled(false);
        act_auto_->setEnabled(false);
        act_stop_->setEnabled(true);
        act_fault_->setEnabled(false);
        act_reset_->setEnabled(false);
        act_io_->setEnabled(true);
        break;

    case HmiMode::STOPPED:
        act_manual_->setEnabled(true);
        act_auto_->setEnabled(true);
        act_stop_->setEnabled(false);
        act_fault_->setEnabled(false);
        act_reset_->setEnabled(true);
        act_io_->setEnabled(true);
        break;

    case HmiMode::FAULT:
        act_manual_->setEnabled(false);
        act_auto_->setEnabled(false);
        act_stop_->setEnabled(true);
        act_fault_->setEnabled(true);
        act_reset_->setEnabled(true);
        act_io_->setEnabled(false);
        break;
    }
}

// ————— 系统状态反馈 —————

void MainWindow::onSystemStatusReceived(const QString &status)
{
    // 控制器反馈的状态字符串: "INIT" / "READY" / "MANUAL" / "AUTO" / "STOPPED" / "FAULT"
    HmiMode new_mode;

    if (status == "INIT")          new_mode = HmiMode::INIT;
    else if (status == "READY")    new_mode = HmiMode::READY;
    else if (status == "MANUAL")   new_mode = HmiMode::MANUAL;
    else if (status == "AUTO")     new_mode = HmiMode::AUTO;
    else if (status == "STOPPED")  new_mode = HmiMode::STOPPED;
    else if (status == "FAULT")    new_mode = HmiMode::FAULT;
    else return; // 未知状态，忽略

    // 只有当控制器反馈与当前 HMI 模式不一致时才纠正
    // (用户操作时 HMI 已乐观更新，控制器确认后无需重复)
    if (new_mode != hmi_mode_)
    {
        QString prev = status_label_->text();
        setHmiMode(new_mode);
        onLogMessage(QString("[STATUS] 控制器反馈: %1, HMI 已同步 (原: %2)")
                         .arg(status, prev));
    }
}

// ————— Slots —————

void MainWindow::onEnterManual()
{
    // 乐观更新：立即切换到 MANUAL 模式
    setHmiMode(HmiMode::MANUAL);
    tab_widget_->setCurrentIndex(0);
    bridge_->publishControlCommand("start_manual");
    onLogMessage("[MODE] → Manual (已发送 start_manual)");
}

void MainWindow::onEnterAuto()
{
    // 乐观更新：立即切换到 AUTO 模式
    setHmiMode(HmiMode::AUTO);
    tab_widget_->setCurrentIndex(1);
    bridge_->publishControlCommand("start_auto");
    onLogMessage("[MODE] → Auto (已发送 start_auto)");
}

void MainWindow::onStopAll()
{
    bridge_->publishControlCommand("stop");
    bridge_->publishJogStopAll();
    // 乐观更新：切换到 STOPPED
    setHmiMode(HmiMode::STOPPED);
    onLogMessage("[CMD] 停机 - 等待轴回到就绪状态");
}

void MainWindow::onClearFault()
{
    bridge_->publishControlCommand("clear_fault");
    // 乐观更新：清除故障后预期回到 INIT → READY
    setHmiMode(HmiMode::INIT);
    onLogMessage("[CMD] Clear fault - 等待轴恢复就绪");
}

void MainWindow::onResetAxes()
{
    bridge_->publishControlCommand("reset");
    // 重置后回到初始化
    setHmiMode(HmiMode::INIT);
    onLogMessage("[CMD] Reset axes - 轴将重新初始化");
}

void MainWindow::onResetAllIo()
{
    // 仅复位 DO 输出信号 (DI 输入信号不可控制)
    for (const auto &dout : bridge_->getDoSignals()) {
        bridge_->publishIoControl(dout.address, 0);
    }
    onLogMessage("[IO] 所有 DO 输出已置为 OFF");
}

void MainWindow::onVirtualStart()
{
    bridge_->publishVirtualIo("virtual_start");
}

void MainWindow::onVirtualEstop()
{
    bridge_->publishVirtualIo("virtual_estop");
}

void MainWindow::onLogMessage(const QString &msg)
{
    QString timestamp = QDateTime::currentDateTime().toString("HH:mm:ss");
    log_panel_->append(QString("[%1] %2").arg(timestamp, msg));
}
