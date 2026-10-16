#include "main_window.hpp"

#include <QAction>
#include <QDateTime>
#include <QGridLayout>
#include <QMessageBox>
#include <QStringList>
#include <QSplitter>
#include <QVBoxLayout>

#include <map>

namespace {
/// 回原设置面板的背景色，与虚拟按钮面板区分
constexpr const char *kHomingPanelStyle =
    "#homingPanel { background-color: #eef4ff; border-bottom: 2px solid #b9cdf0; }";
}

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
        "min-height: 36px; min-width: 100px; font-size: 14px; border-radius: 4px; }"
        "QPushButton:hover { background-color: #45a049; }");
    vLayout->addWidget(btn_virtual_start_);

    // 复位（按住3秒确认，与实体复位按钮语义一致）
    btn_virtual_reset_ = new QPushButton("⟳ 虚拟复位(3s)", virtual_panel_);
    btn_virtual_reset_->setStyleSheet(
        "QPushButton { background-color: #FF9800; color: white; font-weight: bold;"
        "min-height: 36px; min-width: 100px; font-size: 14px; border-radius: 4px; }"
        "QPushButton:hover { background-color: #F57C00; }");
    vLayout->addWidget(btn_virtual_reset_);

    // 虚拟暂停与实体暂停共享同一个 toggle 状态（点一次 = 暂停，再点 = 恢复）
    btn_virtual_pause_ = new QPushButton("⏸ 虚拟暂停", virtual_panel_);
    btn_virtual_pause_->setStyleSheet(
        "QPushButton { background-color: #2196F3; color: white; font-weight: bold;"
        "min-height: 36px; min-width: 100px; font-size: 14px; border-radius: 4px; }"
        "QPushButton:hover { background-color: #1976D2; }");
    vLayout->addWidget(btn_virtual_pause_);
    syncVirtualPauseButton();

    btn_virtual_estop_ = new QPushButton("■ 虚拟急停", virtual_panel_);
    vLayout->addWidget(btn_virtual_estop_);
    syncVirtualEstopButton();

    // 按下/释放事件对（与实体按钮按压语义一致，支持复位3秒长按）
    connect(btn_virtual_start_, &QPushButton::pressed,  this, &MainWindow::onVirtualStartPressed);
    connect(btn_virtual_start_, &QPushButton::released, this, &MainWindow::onVirtualStartReleased);
    connect(btn_virtual_reset_,  &QPushButton::pressed,  this, &MainWindow::onVirtualResetPressed);
    connect(btn_virtual_reset_,  &QPushButton::released, this, &MainWindow::onVirtualResetReleased);
    connect(btn_virtual_pause_,  &QPushButton::pressed,  this, &MainWindow::onVirtualPausePressed);
    connect(btn_virtual_pause_,  &QPushButton::released, this, &MainWindow::onVirtualPauseReleased);
    connect(btn_virtual_estop_,  &QPushButton::clicked,   this, &MainWindow::onVirtualEstop);

    // 回原设置面板（放在虚拟按钮下方）
    setupHomingPanel();

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
    mainLayout->addWidget(homing_panel_);
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
    renderModeUi(mode);
}

// 按模式渲染界面（状态栏 + 页签 + 工具栏门控）。
// 不做"模式未变则早退"的判断 —— 解除暂停后必须无条件重刷：否则 setHmiMode 会早退，
// 状态栏会卡在"暂停中"、按钮门控也留在暂停期间的"可预选(启用)"状态，
// 表现就是"能点手/自动，但点了不生效"（命令会落到状态机里滞留）。
void MainWindow::renderModeUi(HmiMode mode)
{
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

    // ---- 页签跟随（方案A）---------------------------------------------
    // 由 renderModeUi 统一负责，使【按钮点击(乐观更新)】与【控制器反馈】两条来源一致：
    // 控制器经 /system_status 反馈 MANUAL/AUTO 时、以及解除暂停强制重刷时，页签都会切到对应页。
    // setHmiMode 只在模式"变化"时调用本函数，故正常流程不会反复抢页签。
    // READY / STOPPED / INIT / FAULT 不切页，仍由操作员自行选择查看。
    if (tab_widget_)
    {
        if (mode == HmiMode::MANUAL)     { tab_widget_->setCurrentIndex(0); }
        else if (mode == HmiMode::AUTO)  { tab_widget_->setCurrentIndex(1); }
    }

    syncToolbarState();
    syncHomingPanelState();
}

// 暂停态渲染：状态栏显示"暂停中"与本地预选状态，并按"只能预选一次"刷新按钮门控。
// 说明：不修改 hmi_mode_（避免污染乐观更新与回原门控），只在暂停期间覆盖显示与门控。
void MainWindow::applyPausedUiState()
{
    QString pendText;
    switch (pending_mode_local_)
    {
    case 1:  pendText = "已预选 手动（解除暂停后生效）"; break;
    case 2:  pendText = "已预选 自动（解除暂停后生效）"; break;
    default: pendText = "可预选手动/自动（解除暂停后生效）"; break;
    }

    mode_label_->setText("模式: 暂停中");
    status_label_->setText(QString("状态: %1").arg(pendText));

    syncToolbarState();
}

void MainWindow::syncToolbarState()
{
    if (!act_manual_ || !act_auto_ || !act_stop_ ||
        !act_fault_ || !act_reset_ || !act_io_)
        return;

    // 暂停态优先（控制器上报 "|PAUSED"）：
    //   暂停中轴停在 READY，此时允许"预选"一次手/自动（解除暂停时生效）；
    //   预选过一次后两个模式按钮即禁用 —— 效果上等价于"只能选一次"。
    //   注意：这是 HMI 本地门控，控制器侧不做"忽略第二次"（按既定决策）。
    if (controller_paused_)
    {
        const bool can_preselect = (pending_mode_local_ == 0);
        act_manual_->setEnabled(can_preselect);
        act_auto_->setEnabled(can_preselect);
        act_stop_->setEnabled(false);
        act_fault_->setEnabled(false);
        act_reset_->setEnabled(false);
        act_io_->setEnabled(false);
        return;
    }

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
    // 控制器反馈格式："INIT/READY/MANUAL/AUTO/STOPPED/FAULT"，暂停时追加 "|PAUSED"
    // （见 ethercat_controller 的 publish_system_status）。
    // 同一话题上的其它文本（"执行命令: xxx"、点动速度提示）不含 '|'，会在下面被忽略。
    const int sep = status.indexOf('|');
    const QString state = (sep < 0) ? status : status.left(sep);
    const QString flags = (sep < 0) ? QString() : status.mid(sep + 1);
    const bool paused        = flags.contains("PAUSED");
    const bool estop_latched = flags.contains("ESTOP");

    HmiMode new_mode;

    if (state == "INIT")          new_mode = HmiMode::INIT;
    else if (state == "READY")    new_mode = HmiMode::READY;
    else if (state == "MANUAL")   new_mode = HmiMode::MANUAL;
    else if (state == "AUTO")     new_mode = HmiMode::AUTO;
    else if (state == "STOPPED")  new_mode = HmiMode::STOPPED;
    else if (state == "FAULT")    new_mode = HmiMode::FAULT;
    else return; // 未知状态，忽略

    // 回原设置门控必须以控制器真实回报的状态为准（不能用乐观更新的 hmi_mode_）
    controller_reported_mode_ = new_mode;
    if (new_mode == HmiMode::INIT) {
        // 轴重新初始化后，之前的"已保存待生效"提示失效
        homing_pending_apply_ = false;
    }

    // ---- 虚拟急停锁存：显示跟随控制器上报（避免 HMI 重启/多客户端导致错位）----
    // 注：上报的是"虚拟急停软件锁存" g_virtual_estop_active，与实体急停无关，
    //     因此实体急停不会点亮这个按钮。
    if (estop_latched != virtual_estop_latched_)
    {
        virtual_estop_latched_ = estop_latched;
        syncVirtualEstopButton();
        onLogMessage(estop_latched
                         ? "[VirtualIO] 控制器虚拟急停锁存：已激活（HMI 已同步）"
                         : "[VirtualIO] 控制器虚拟急停锁存：已解除（HMI 已同步）");
    }

    // ---- 暂停态：只刷新"暂停中/已预选"的显示与门控，不把乐观选择拉回去 ----
    // 暂停时轴停在 READY，若走下面的常规纠正，会把用户刚点的"自动"拉回"就绪"，
    // 于是操作员看不到自己预选了什么。暂停期间的显示交给 applyPausedUiState()。
    if (paused)
    {
        if (!controller_paused_)
        {
            controller_paused_ = true;
            pending_mode_local_ = 0;   // 进入暂停：本地预选归零
            onLogMessage("[PAUSE] 控制器进入暂停态，可预选手/自动（解除后生效）");
            syncVirtualPauseButton();  // 方案A：虚拟暂停按钮跟随真实状态
        }
        applyPausedUiState();
        syncHomingPanelState();
        return;
    }

    // ---- 非暂停 ----
    if (controller_paused_)
    {
        // 刚解除暂停：清本地预选标记，并**强制**重刷模式 UI。
        // 必须强制：setHmiMode 在"模式未变"时会早退（暂停前是自动 + 预选自动 → 恢复后仍是自动），
        // 那样状态栏会卡在"暂停中"、按钮门控会留在暂停期间的"可预选(启用)"状态，
        // 于是表现成"还能点手/自动，但点了不生效"（命令落到状态机里滞留）。
        controller_paused_ = false;
        pending_mode_local_ = 0;
        hmi_mode_ = new_mode;
        renderModeUi(new_mode);
        syncVirtualPauseButton();      // 方案A：虚拟暂停按钮跟随真实状态
        onLogMessage(QString("[PAUSE] 解除暂停，控制器模式: %1").arg(state));
    }
    // 只有当控制器反馈与当前 HMI 模式不一致时才纠正
    // (用户操作时 HMI 已乐观更新，控制器确认后无需重复)
    else if (new_mode != hmi_mode_)
    {
        QString prev = status_label_->text();
        setHmiMode(new_mode);
        onLogMessage(QString("[STATUS] 控制器反馈: %1, HMI 已同步 (原: %2)")
                         .arg(state, prev));
    }

    syncHomingPanelState();
}

// ————— Slots —————

void MainWindow::onEnterManual()
{
    // 暂停中：只"预选"，不改界面模式（控制器会记录，解除暂停时生效），
    //         并立刻刷新为"已预选"，同时锁住两个模式按钮（等价"只能选一次"）。
    if (controller_paused_)
    {
        pending_mode_local_ = 1;
        bridge_->publishControlCommand("start_manual");
        onLogMessage("[MODE] 暂停中 → 已预选 手动（解除暂停后生效）");
        applyPausedUiState();
        return;
    }

    // 乐观更新：立即切换到 MANUAL 模式（页签切换已收进 setHmiMode，避免两处来源）
    setHmiMode(HmiMode::MANUAL);
    bridge_->publishControlCommand("start_manual");
    onLogMessage("[MODE] → Manual (已发送 start_manual)");
}

void MainWindow::onEnterAuto()
{
    // 暂停中：只"预选"，见 onEnterManual 的说明
    if (controller_paused_)
    {
        pending_mode_local_ = 2;
        bridge_->publishControlCommand("start_auto");
        onLogMessage("[MODE] 暂停中 → 已预选 自动（解除暂停后生效）");
        applyPausedUiState();
        return;
    }

    // 乐观更新：立即切换到 AUTO 模式（页签切换已收进 setHmiMode，避免两处来源）
    setHmiMode(HmiMode::AUTO);
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
    // 遍历长度 = io_config.yaml 中 do_signals 的条数（已按控制器上限截断），
    // 因此不会打到控制器范围外的地址上
    const auto &do_signals = bridge_->getDoSignals();
    for (const auto &dout : do_signals) {
        bridge_->publishIoControl(dout.address, 0);
    }
    onLogMessage(QString("[IO] 所有 DO 输出已置为 OFF（共 %1 个）").arg(do_signals.size()));
}

// ————— 虚拟按钮（按下/释放事件对，与实体按钮按压语义一致） —————

void MainWindow::onVirtualStartPressed()
{
    bridge_->publishVirtualIo("virtual_start_down");
}

void MainWindow::onVirtualStartReleased()
{
    bridge_->publishVirtualIo("virtual_start_up");
}

void MainWindow::onVirtualResetPressed()
{
    bridge_->publishVirtualIo("virtual_reset_down");
}

void MainWindow::onVirtualResetReleased()
{
    bridge_->publishVirtualIo("virtual_reset_up");
}

void MainWindow::onVirtualPausePressed()
{
    // 方案A：按钮显示不再本地"数点击次数"，而是跟随控制器上报的 "|PAUSED"。
    // 这里只发命令；按下后 ≤100ms（下一帧 /system_status）由 syncVirtualPauseButton() 刷新。
    // 这样无论用实体暂停还是虚拟暂停，按钮显示都与真实暂停态一致，不会再错位。
    bridge_->publishVirtualIo("virtual_pause_down");
}

void MainWindow::onVirtualPauseReleased()
{
    // 必须发 up：控制器侧是电平边沿检测，释放后回落，下一次按下才能再产生上升沿
    bridge_->publishVirtualIo("virtual_pause_up");
}

// 虚拟急停：点击切换锁存，再点一次解除。
// 锁存状态仍由控制器持有（g_virtual_estop_active），界面这里只发命令；
// 按钮的"激活/解除"显示跟随控制器上报的 "|ESTOP"（见 onSystemStatusReceived），
// 因此不再本地翻转 —— 这样 HMI 重启、多客户端操作也不会与真实状态错位。
void MainWindow::onVirtualEstop()
{
    bridge_->publishVirtualIo("virtual_estop");
    onLogMessage("[VirtualIO] 已发送虚拟急停切换命令（等待控制器确认）");
}

// 同步虚拟急停按钮显示（锁存时高亮提示，避免用户遗忘导致无法启动）
// 判据 virtual_estop_latched_ 由控制器上报的 "|ESTOP" 驱动（不再本地翻转）。
void MainWindow::syncVirtualEstopButton()
{
    if (virtual_estop_latched_) {
        btn_virtual_estop_->setText("■ 急停激活(点击解除)");
        btn_virtual_estop_->setStyleSheet(
            "QPushButton { background-color: #B71C1C; color: white; font-weight: bold;"
            "min-height: 36px; min-width: 100px; font-size: 14px; border-radius: 4px;"
            "border: 2px solid #FFEB3B; }");
    } else {
        btn_virtual_estop_->setText("■ 虚拟急停");
        btn_virtual_estop_->setStyleSheet(
            "QPushButton { background-color: #E53935; color: white; font-weight: bold;"
            "min-height: 36px; min-width: 100px; font-size: 14px; border-radius: 4px; }"
            "QPushButton:hover { background-color: #c62828; }");
    }
}

// 同步虚拟暂停按钮显示（暂停中高亮，提示"再点一次恢复"）
// 状态源唯一：控制器 /system_status 上报的 "|PAUSED"（controller_paused_）。
// 不再用"本界面点了几次"的本地镜像 —— 否则用实体暂停按钮切换时，按钮会与实际状态错位。
void MainWindow::syncVirtualPauseButton()
{
    if (controller_paused_) {
        btn_virtual_pause_->setText("⏸ 暂停中(点击恢复)");
        btn_virtual_pause_->setStyleSheet(
            "QPushButton { background-color: #B71C1C; color: white; font-weight: bold;"
            "min-height: 36px; min-width: 100px; font-size: 14px; border-radius: 4px;"
            "border: 2px solid #FFEB3B; }");
    } else {
        btn_virtual_pause_->setText("⏸ 虚拟暂停");
        btn_virtual_pause_->setStyleSheet(
            "QPushButton { background-color: #2196F3; color: white; font-weight: bold;"
            "min-height: 36px; min-width: 100px; font-size: 14px; border-radius: 4px; }"
            "QPushButton:hover { background-color: #1976D2; }");
    }
}

void MainWindow::onLogMessage(const QString &msg)
{
    QString timestamp = QDateTime::currentDateTime().toString("HH:mm:ss");
    log_panel_->append(QString("[%1] %2").arg(timestamp, msg));
}

// ————— 回原设置面板 —————

void MainWindow::setupHomingPanel()
{
    homing_panel_ = new QFrame(this);
    homing_panel_->setObjectName("homingPanel");
    homing_panel_->setFrameStyle(QFrame::StyledPanel | QFrame::Plain);
    homing_panel_->setStyleSheet(kHomingPanelStyle);

    auto *vLayout = new QVBoxLayout(homing_panel_);
    vLayout->setContentsMargins(8, 4, 8, 6);
    vLayout->setSpacing(4);

    // 标题行：标题 + 状态提示 + 操作按钮
    auto *titleRow = new QHBoxLayout();
    auto *title = new QLabel("回原设置", homing_panel_);
    title->setStyleSheet("font-weight: bold; font-size: 14px;");
    titleRow->addWidget(title);

    homing_hint_label_ = new QLabel("", homing_panel_);
    homing_hint_label_->setStyleSheet("color: #888; font-size: 12px;");
    titleRow->addWidget(homing_hint_label_);
    titleRow->addStretch();

    btn_homing_reload_  = new QPushButton("重新载入", homing_panel_);
    btn_homing_factory_ = new QPushButton("恢复出厂", homing_panel_);
    btn_homing_apply_   = new QPushButton("应用", homing_panel_);
    btn_homing_apply_->setStyleSheet(
        "QPushButton { background-color: #1976D2; color: white; font-weight: bold;"
        " min-height: 28px; min-width: 80px; border-radius: 4px; }"
        "QPushButton:hover { background-color: #1565C0; }"
        "QPushButton:disabled { background-color: #bbb; color: #eee; }");
    titleRow->addWidget(btn_homing_reload_);
    titleRow->addWidget(btn_homing_factory_);
    titleRow->addWidget(btn_homing_apply_);
    vLayout->addLayout(titleRow);

    // 轴列表：每行 2 个轴，用下拉框做显式二值选择（回原 / 不回原）
    auto *grid = new QGridLayout();
    grid->setHorizontalSpacing(16);
    grid->setVerticalSpacing(2);

    const int n = bridge_->getNumAxes();
    homing_combos_.clear();
    homing_combos_.reserve(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
        const int row = i / 2;
        const int col = (i % 2) * 2;

        auto *lab = new QLabel(
            QString("%1 (%2)")
                .arg(QString::fromStdString(bridge_->getDisplayName(i)),
                     QString::fromStdString(bridge_->getAxisName(i))),
            homing_panel_);

        auto *cb = new QComboBox(homing_panel_);
        cb->addItem("不回原 (CSP)", 0);
        cb->addItem("回原 (HM)", 1);
        cb->setCurrentIndex(bridge_->getAxisHomingValue(i) ? 1 : 0);
        if (!bridge_->isAxisHomingCapable(i)) {
            cb->setToolTip("该品牌未列入回原白名单，请确认驱动器是否支持 HM 回原模式");
        }

        grid->addWidget(lab, row, col);
        grid->addWidget(cb,  row, col + 1);
        homing_combos_.push_back(cb);
    }
    grid->setColumnStretch(4, 1);
    vLayout->addLayout(grid);

    connect(btn_homing_apply_,   &QPushButton::clicked, this, &MainWindow::onApplyHomingConfig);
    connect(btn_homing_reload_,  &QPushButton::clicked, this, &MainWindow::onReloadHomingConfig);
    connect(btn_homing_factory_, &QPushButton::clicked, this, &MainWindow::onRestoreFactoryHoming);

    syncHomingPanelState();
}

void MainWindow::refreshHomingPanelFromBridge()
{
    const int n = bridge_->getNumAxes();
    for (size_t i = 0; i < homing_combos_.size(); ++i) {
        const int idx = static_cast<int>(i);
        if (idx >= n) break;
        homing_combos_[i]->setCurrentIndex(bridge_->getAxisHomingValue(idx) ? 1 : 0);
    }
}

bool MainWindow::canEditHoming() const
{
    // 只有控制器明确回报"就绪/停机"才允许修改：
    // 此时 init_axes() 已结束、YAML 已读完，写文件不会与控制器读盘竞争
    return controller_reported_mode_ == HmiMode::READY ||
           controller_reported_mode_ == HmiMode::STOPPED;
}

void MainWindow::syncHomingPanelState()
{
    if (!homing_panel_) return;

    const bool editable = canEditHoming();
    for (auto *cb : homing_combos_) cb->setEnabled(editable);
    btn_homing_apply_->setEnabled(editable);
    btn_homing_reload_->setEnabled(editable);
    btn_homing_factory_->setEnabled(editable);

    if (!editable) {
        homing_hint_label_->setText(QString::fromUtf8("⚠ 设备未就绪，不可修改（需先进入“就绪/停机”状态）"));
        homing_hint_label_->setStyleSheet("color: #888; font-size: 12px;");
    } else if (homing_pending_apply_) {
        homing_hint_label_->setText(QString::fromUtf8("✔ 已保存，待生效：请长按复位按钮 3 秒"));
        homing_hint_label_->setStyleSheet("color: #B26A00; font-size: 12px; font-weight: bold;");
    } else {
        homing_hint_label_->setText(QString::fromUtf8("可修改：每轴可单独设置是否回原"));
        homing_hint_label_->setStyleSheet("color: #2E7D32; font-size: 12px;");
    }
}

void MainWindow::onApplyHomingConfig()
{
    if (!canEditHoming()) {
        QMessageBox::warning(this, "回原设置",
                             "当前设备状态不允许修改回原配置。\n请等待轴就绪，或先停机。");
        return;
    }

    std::map<std::string, bool> flags;
    QStringList homing_names;
    const int n = bridge_->getNumAxes();
    for (size_t i = 0; i < homing_combos_.size(); ++i) {
        const int idx = static_cast<int>(i);
        if (idx >= n) break;
        const bool want = (homing_combos_[i]->currentIndex() == 1);
        flags[bridge_->getAxisName(idx)] = want;
        if (want) homing_names << QString::fromStdString(bridge_->getDisplayName(idx));
    }

    QString msg;
    if (homing_names.isEmpty()) {
        msg = QString::fromUtf8("将把所有轴设为【不回原】。\n\n保存后需长按复位按钮 3 秒才生效。");
    } else {
        msg = QString::fromUtf8("将把以下轴设为【回原轴】：\n  %1\n\n"
                                "⚠ 多轴同时回原存在机械干涉风险，请确认设备状态。\n"
                                "保存后需长按复位按钮 3 秒才生效。")
                  .arg(homing_names.join("、"));
    }

    if (QMessageBox::question(this, "确认回原配置", msg,
                              QMessageBox::Yes | QMessageBox::No,
                              QMessageBox::No) != QMessageBox::Yes) {
        return;
    }

    if (bridge_->saveAxesHomingConfig(flags)) {
        homing_pending_apply_ = true;
        refreshHomingPanelFromBridge();
        syncHomingPanelState();
        onLogMessage("[HOMING] 回原配置已保存，等待长按复位按钮生效");
    } else {
        QMessageBox::critical(this, "回原设置", "写入失败，请查看日志了解原因。");
    }
}

void MainWindow::onReloadHomingConfig()
{
    if (bridge_->reloadAxesConfig()) {
        refreshHomingPanelFromBridge();
        homing_pending_apply_ = false;
        syncHomingPanelState();
        onLogMessage("[HOMING] 已重新载入 axes_config.yaml");
    } else {
        QMessageBox::critical(this, "回原设置", "重新载入失败，请查看日志。");
    }
}

void MainWindow::onRestoreFactoryHoming()
{
    if (!canEditHoming()) {
        QMessageBox::warning(this, "回原设置", "当前设备状态不允许修改回原配置。");
        return;
    }

    if (QMessageBox::question(this, "恢复出厂回原配置",
                              QString::fromUtf8("将用出厂快照 (.factory) 覆盖 axes_config.yaml。\n\n是否继续？"),
                              QMessageBox::Yes | QMessageBox::No,
                              QMessageBox::No) != QMessageBox::Yes) {
        return;
    }

    if (bridge_->restoreAxesHomingFactory()) {
        refreshHomingPanelFromBridge();
        homing_pending_apply_ = false;
        syncHomingPanelState();
        onLogMessage("[HOMING] 已恢复出厂回原配置，需长按复位按钮生效");
    } else {
        QMessageBox::critical(this, "回原设置", "恢复出厂失败，请查看日志。");
    }
}
