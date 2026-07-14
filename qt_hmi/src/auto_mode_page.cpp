#include "auto_mode_page.hpp"

#include <QGroupBox>
#include <QGridLayout>

AutoModePage::AutoModePage(std::shared_ptr<Ros2Bridge> bridge, QWidget *parent)
    : QWidget(parent), bridge_(bridge)
{
    // 连接关节状态信号 → 实时更新各轴位置
    connect(bridge_.get(), &Ros2Bridge::jointStatesUpdated,
            this, &AutoModePage::onJointStatesUpdated);

    int num_axes = bridge_->getNumAxes();

    // 1. 轴位置控制区: 动态 AxisPanel
    auto *axis_area = new QScrollArea(this);
    auto *axis_container = new QWidget();
    auto *axis_grid = new QGridLayout(axis_container);

    for (int i = 0; i < num_axes; ++i) {
        std::string axis_name     = bridge_->getAxisName(i);
        std::string calc_mode     = bridge_->getCalcMode(i);
        std::string display_name  = bridge_->getDisplayName(i);

        auto *panel = new AxisPanel(axis_name, calc_mode, display_name, bridge_, this);
        axis_panels_.push_back(panel);

        // 2列布局: 每行放2个面板
        int row = i / 2;
        int col = i % 2;
        axis_grid->addWidget(panel, row, col);
    }

    axis_area->setWidget(axis_container);
    axis_area->setWidgetResizable(true);

    // 2. IO 控制
    io_panel_ = new IoPanel(bridge_, this);

    // — 左右分割布局 —
    // 左侧: 轴位置控制
    // 右侧: IO控制
    auto *left_widget = axis_area;

    auto *main_splitter = new QSplitter(Qt::Horizontal, this);
    main_splitter->addWidget(left_widget);
    main_splitter->addWidget(io_panel_);
    main_splitter->setStretchFactor(0, 3);
    main_splitter->setStretchFactor(1, 2);

    auto *layout = new QVBoxLayout(this);
    layout->addWidget(main_splitter);
}

void AutoModePage::onIoStatusUpdated(const std::string &address, int value)
{
    io_panel_->onIoStatusUpdated(address, value);
}

void AutoModePage::onJointStatesUpdated(const std::vector<std::string> &names,
                                        const std::vector<double> &positions)
{
    for (size_t i = 0; i < names.size() && i < positions.size(); ++i) {
        for (auto *panel : axis_panels_) {
            if (panel->axisName() == names[i]) {
                panel->updateJointPosition(positions[i]);
                break;
            }
        }
    }
}
