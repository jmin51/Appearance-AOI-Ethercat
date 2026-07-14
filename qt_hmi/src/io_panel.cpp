#include "io_panel.hpp"

#include <QFrame>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QScrollArea>

// ————— 辅助：创建带标题的边框区域 —————
static QFrame *makeSectionFrame(const QString &title, QWidget *parent)
{
    auto *frame = new QFrame(parent);
    frame->setFrameStyle(QFrame::StyledPanel | QFrame::Plain);
    frame->setStyleSheet("QFrame { border: 1px solid #ccc; border-radius: 2px; }");
    return frame;
}

static QLabel *makeSectionTitle(const QString &text, QWidget *parent)
{
    auto *lbl = new QLabel(text, parent);
    lbl->setAlignment(Qt::AlignCenter);
    lbl->setStyleSheet(
        "QLabel { font-weight: bold; font-size: 14px; min-height: 24px; "
        "background-color: #e8e8e8; border-bottom: 1px solid #ccc; }");
    return lbl;
}

IoPanel::IoPanel(std::shared_ptr<Ros2Bridge> bridge, QWidget *parent)
    : QWidget(parent), bridge_(bridge)
{
    // 主外框
    auto *frame = new QFrame(this);
    frame->setFrameStyle(QFrame::StyledPanel | QFrame::Plain);
    frame->setStyleSheet("QFrame { border: 1px solid #aaa; border-radius: 3px; }");

    auto *vbox = new QVBoxLayout(frame);
    vbox->setContentsMargins(4, 2, 4, 4);
    vbox->setSpacing(4);

    // 顶部标题
    auto *mainTitle = new QLabel("远程IO模块控制");
    mainTitle->setAlignment(Qt::AlignCenter);
    mainTitle->setStyleSheet("QLabel { font-weight: bold; font-size: 17px; min-height: 32px; }");
    vbox->addWidget(mainTitle);

    // ———— 左右并排布局 ————
    auto *hbox = new QHBoxLayout();
    hbox->setSpacing(6);

    const auto &di_list = bridge_->getDiSignals();
    const auto &do_list = bridge_->getDoSignals();

    // ============ DI 输入区域 (左侧, 只读) ============
    auto *diFrame  = makeSectionFrame("DI", this);
    auto *diLayout = new QVBoxLayout(diFrame);
    diLayout->setContentsMargins(0, 0, 0, 0);
    diLayout->setSpacing(0);

    auto *diTitle  = makeSectionTitle("DI 输入信号 (只读)", this);
    diLayout->addWidget(diTitle);

    auto *diGrid = new QGridLayout();
    diGrid->setContentsMargins(4, 2, 4, 2);
    diGrid->setVerticalSpacing(1);
    diGrid->setHorizontalSpacing(4);

    // DI 表头
    auto *diHdrAddr = new QLabel("地址");
    auto *diHdrName = new QLabel("信号名称");
    auto *diHdrState = new QLabel("状态");
    diHdrAddr->setStyleSheet("QLabel { font-weight: bold; color: #555; min-height: 20px; }");
    diHdrName->setStyleSheet("QLabel { font-weight: bold; color: #555; min-height: 20px; }");
    diHdrState->setStyleSheet("QLabel { font-weight: bold; color: #555; min-height: 20px; }");
    diGrid->addWidget(diHdrAddr, 0, 0);
    diGrid->addWidget(diHdrName, 0, 1);
    diGrid->addWidget(diHdrState, 0, 2);
    diGrid->setColumnStretch(1, 1);

    int diRow = 1;
    for (const auto &sig : di_list) {
        IoItem item;
        item.address = sig.address;
        item.current_value = 0;
        item.is_input = true;

        // 地址标签: "M500"
        auto *addrLabel = new QLabel(QString("M%1").arg(sig.address.c_str()), this);
        addrLabel->setStyleSheet("QLabel { font-family: monospace; color: #333; min-width: 42px; }");

        // 名称标签
        item.name_label = new QLabel(QString::fromStdString(sig.desc), this);
        item.name_label->setStyleSheet("QLabel { padding-left: 4px; }");

        // LED 指示灯
        item.led = new QLabel("⚫", this);
        item.led->setAlignment(Qt::AlignCenter);
        item.led->setFixedSize(18, 18);
        item.led->setStyleSheet("QLabel { font-size: 12px; }");

        diGrid->addWidget(addrLabel,       diRow, 0);
        diGrid->addWidget(item.name_label, diRow, 1);
        diGrid->addWidget(item.led,        diRow, 2);

        di_items_.push_back(item);
        diRow++;
    }
    diGrid->setRowStretch(diRow, 1);
    diLayout->addLayout(diGrid);

    // DI 区域可滚动
    auto *diScroll = new QScrollArea(this);
    diScroll->setWidget(diFrame);
    diScroll->setWidgetResizable(true);
    diScroll->setFrameShape(QFrame::NoFrame);
    diScroll->setMinimumHeight(260);

    // ============ DO 输出区域 (右侧, 可控) ============
    auto *doFrame  = makeSectionFrame("DO", this);
    auto *doLayout = new QVBoxLayout(doFrame);
    doLayout->setContentsMargins(0, 0, 0, 0);
    doLayout->setSpacing(0);

    auto *doTitle  = makeSectionTitle("DO 输出控制 (可控)", this);
    doLayout->addWidget(doTitle);

    auto *doGrid = new QGridLayout();
    doGrid->setContentsMargins(4, 2, 4, 2);
    doGrid->setVerticalSpacing(1);
    doGrid->setHorizontalSpacing(4);

    // DO 表头
    auto *doHdrAddr = new QLabel("地址");
    auto *doHdrBtn  = new QLabel("切换");
    auto *doHdrState = new QLabel("状态");
    doHdrAddr->setStyleSheet("QLabel { font-weight: bold; color: #555; min-height: 20px; }");
    doHdrBtn->setStyleSheet("QLabel { font-weight: bold; color: #555; min-height: 20px; }");
    doHdrState->setStyleSheet("QLabel { font-weight: bold; color: #555; min-height: 20px; }");
    doGrid->addWidget(doHdrAddr,  0, 0);
    doGrid->addWidget(doHdrBtn,   0, 1);
    doGrid->addWidget(doHdrState, 0, 2);
    doGrid->setColumnStretch(1, 1);

    int doRow = 1;
    for (const auto &sig : do_list) {
        IoItem item;
        item.address = sig.address;
        item.current_value = 0;
        item.is_input = false;

        // 地址标签
        auto *addrLabel = new QLabel(QString("M%1").arg(sig.address.c_str()), this);
        addrLabel->setStyleSheet("QLabel { font-family: monospace; color: #333; min-width: 42px; }");

        // 切换按钮
        item.button = new QPushButton(QString::fromStdString(sig.desc), this);
        item.button->setStyleSheet(
            "QPushButton { min-height: 20px; text-align: left; padding-left: 4px; }");

        // LED 指示灯
        item.led = new QLabel("⚫", this);
        item.led->setAlignment(Qt::AlignCenter);
        item.led->setFixedSize(18, 18);
        item.led->setStyleSheet("QLabel { font-size: 12px; }");

        connect(item.button, &QPushButton::clicked, this,
                [this, addr = sig.address]() { onToggleIo(addr); });

        doGrid->addWidget(addrLabel,   doRow, 0);
        doGrid->addWidget(item.button, doRow, 1);
        doGrid->addWidget(item.led,    doRow, 2);

        do_items_.push_back(item);
        doRow++;
    }

    // DO 复位按钮
    reset_all_btn_ = new QPushButton("复位所有 DO", this);
    reset_all_btn_->setStyleSheet(
        "QPushButton { background-color: #FF9800; color: white; font-weight: bold; "
        "min-height: 26px; margin: 4px; }");
    connect(reset_all_btn_, &QPushButton::clicked, this, &IoPanel::onResetAll);
    doGrid->addWidget(reset_all_btn_, doRow, 0, 1, 3);
    doGrid->setRowStretch(doRow + 1, 1);
    doLayout->addLayout(doGrid);

    auto *doScroll = new QScrollArea(this);
    doScroll->setWidget(doFrame);
    doScroll->setWidgetResizable(true);
    doScroll->setFrameShape(QFrame::NoFrame);
    doScroll->setMinimumHeight(260);

    // 将左右区域加入水平布局
    hbox->addWidget(diScroll, 1);
    hbox->addWidget(doScroll, 1);
    vbox->addLayout(hbox, 1);

    auto *outer = new QVBoxLayout(this);
    outer->setContentsMargins(0, 0, 0, 0);
    outer->addWidget(frame);

    // 连接 Ros2Bridge 信号到面板
    connect(bridge_.get(), &Ros2Bridge::ioStatusUpdated, this, &IoPanel::onIoStatusUpdated);
    connect(bridge_.get(), &Ros2Bridge::diStatusUpdated,  this, &IoPanel::onDiStatusUpdated);
}

// ————— 切换 DO 输出 —————
void IoPanel::onToggleIo(const std::string &address)
{
    for (auto &item : do_items_) {
        if (item.address == address) {
            int new_val = (item.current_value == 0) ? 1 : 0;
            bridge_->publishIoControl(address, new_val);
            break;
        }
    }
}

// ————— DO 状态更新 —————
void IoPanel::onIoStatusUpdated(const std::string &address, int value)
{
    for (auto &item : do_items_) {
        if (item.address == address) {
            item.current_value = value;
            updateLed(item);
            break;
        }
    }
}

// ————— DI 状态更新 —————
void IoPanel::onDiStatusUpdated(const std::string &address, int value)
{
    for (auto &item : di_items_) {
        if (item.address == address) {
            item.current_value = value;
            updateLed(item);
            break;
        }
    }
}

// ————— 复位所有 DO —————
void IoPanel::onResetAll()
{
    for (auto &item : do_items_) {
        if (item.current_value == 1) {
            bridge_->publishIoControl(item.address, 0);
        }
    }
}

// ————— LED 状态更新 —————
void IoPanel::updateLed(IoItem &item)
{
    if (item.current_value == 1) {
        item.led->setText("🟢");
        item.led->setStyleSheet("QLabel { font-size: 12px; color: green; }");
    } else {
        item.led->setText("⚫");
        item.led->setStyleSheet("QLabel { font-size: 12px; color: gray; }");
    }
}
