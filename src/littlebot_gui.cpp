#include <iostream>
#include <cstdint>

#include "littlebot_rqt_plugin/littlebot_gui.hpp"

namespace littlebot_rqt_plugin
{
LittlebotGui::LittlebotGui(QWidget *parent)
    : QDialog(parent)
{
    ui_.setupUi(this);
    this->updateAvailableDevices();
    connect(ui_.push_set_cmd, &QPushButton::clicked, this, &LittlebotGui::sendCommandButtonClicked);
}

void LittlebotGui::sendCommandButtonClicked()
{
    littlebotStatus();
}

void LittlebotGui::littlebotCommand(const std::string &text)
{
    // ui_.textBrowser->append(QString::fromStdString(text));
}

void LittlebotGui::updateAvailableDevices()
{
    ui_.combo_dev_serial_available->clear();

    auto n_ports = available_devices_.scanPorts();

    for (uint16_t i = 0; i <= n_ports; ++i) {
        auto name = available_devices_.findName(i);
        auto port_path = available_devices_.findPortPath(i);
        ui_.combo_dev_serial_available->addItem(QString::fromStdString(*name));
    }
}

}  // namespace littlebot_rqt_plugin