#pragma once

#include <QWidget>
#include <QGridLayout>
#include <QPushButton>
#include <QLabel>
#include <map>
#include <string>
#include <memory>
#include <vector>

#include "ros2_bridge.hpp"

/// IO 控制面板 — DI输入监控 + DO输出控制 + 全复位按钮
/// DI信号 (M500~M515): 只读LED状态显示，不可控制
/// DO信号 (M800~M815): 切换按钮 + LED状态指示，可控制
class IoPanel : public QWidget
{
    Q_OBJECT

public:
    explicit IoPanel(std::shared_ptr<Ros2Bridge> bridge, QWidget *parent = nullptr);

public slots:
    /// 当 Ros2Bridge 收到 /io_status 的 DO 状态后，更新 DO LED 状态
    void onIoStatusUpdated(const std::string &address, int value);
    /// 当 Ros2Bridge 收到 /io_status 的 DI 状态后，更新 DI LED 状态
    void onDiStatusUpdated(const std::string &address, int value);
    /// 复位所有 DO 输出
    void onResetAll();

private slots:
    void onToggleIo(const std::string &address);

private:
    std::shared_ptr<Ros2Bridge> bridge_;

    /// 单个 IO 信号项
    struct IoItem {
        QPushButton *button = nullptr;  // DO有按钮, DI为nullptr
        QLabel *name_label = nullptr;   // DI名称标签 (DO为nullptr)
        QLabel *led = nullptr;          // LED 指示灯
        std::string address;
        int current_value = 0;          // 0=OFF, 1=ON
        bool is_input = false;          // true=DI(只读), false=DO(可控)
    };

    std::vector<IoItem> di_items_;   // DI 输入信号 (只读显示)
    std::vector<IoItem> do_items_;   // DO 输出信号 (可控)
    QPushButton *reset_all_btn_;

    void updateLed(IoItem &item);
};
