#include <iostream>
#include <cstdint>

#include "littlebot_rqt_plugin/littlebot_gui.hpp"

namespace littlebot_rqt_plugin
{
LittlebotGui::LittlebotGui(QWidget *parent)
    : QDialog(parent)
{
    ui_.setupUi(this);
    // this->updateAvailableDevices();
    connect(ui_.push_set_cmd, &QPushButton::clicked, this, &LittlebotGui::sendCommandButtonClicked);

    ui_.combo_dev_serial_available->addItem("Select a device");
    auto n_ports = available_devices_.scanPorts();
    for (uint16_t i = 0; i <= n_ports; ++i) {
        auto name = available_devices_.findName(i);
        auto port_path = available_devices_.findPortPath(i);
        ui_.combo_dev_serial_available->insertItem(i + 1, QString::fromStdString(*name));
    }
    

    connect(ui_.combo_dev_serial_available, QOverload<int>::of(&QComboBox::activated), 
        [this](int index) {this->updateAvailableDevices();
    });
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
    auto number_of_devices = available_devices_.scanPorts();
    if(number_of_devices == current_number_of_devices_) {
        return;
    }
    current_number_of_devices_ = number_of_devices;
    ui_.combo_dev_serial_available->clear();
    std::vector<libserial::Device> devices;
    available_devices_.getDevices(devices);
    for (const auto& device : devices) {
        ui_.combo_dev_serial_available->insertItem(device.getId() + 1, QString::fromStdString(device.getName()));
    }
}

}  // namespace littlebot_rqt_plugin