#include "manual_mode_page.hpp"

ManualModePage::ManualModePage(std::shared_ptr<Ros2Bridge> bridge, QWidget *parent)
    : QWidget(parent), bridge_(bridge)
{
    jog_bar_  = new JogBar(bridge_, this);
    io_panel_ = new IoPanel(bridge_, this);

    // 连接关节状态信号 → 更新 JogBar 每行的位置显示
    connect(bridge_.get(), &Ros2Bridge::jointStatesUpdated,
            this, &ManualModePage::onJointStatesUpdated);

    // 水平分割器: 左侧 JogBar + 右侧 IoPanel
    auto *splitter = new QSplitter(Qt::Horizontal, this);
    splitter->addWidget(jog_bar_);
    splitter->addWidget(io_panel_);
    splitter->setStretchFactor(0, 2);
    splitter->setStretchFactor(1, 3);

    auto *layout = new QVBoxLayout(this);
    layout->addWidget(splitter);
}

void ManualModePage::onIoStatusUpdated(const std::string &address, int value)
{
    // 转发给 IoPanel 更新 LED
    io_panel_->onIoStatusUpdated(address, value);
}

void ManualModePage::onJointStatesUpdated(const std::vector<std::string> &names,
                                          const std::vector<double> &positions)
{
    for (size_t i = 0; i < names.size() && i < positions.size(); ++i) {
        int axis_idx = bridge_->getAxisIndex(names[i]);
        if (axis_idx < 0)
            continue;
        jog_bar_->updatePosition(axis_idx, positions[i]);
    }
}
