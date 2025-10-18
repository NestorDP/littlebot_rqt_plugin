#include <iostream>
#include <cstdint>
#include <stdexcept>
#include <QMessageBox>

#include "littlebot_rqt_plugin/littlebot_gui.hpp"
#include "littlebot_base/littlebot_driver.hpp"
#include "littlebot_base/serial_port.hpp"

namespace littlebot_rqt_plugin
{
LittlebotGui::LittlebotGui(QWidget *parent)
    : QDialog(parent)
{
    ui_.setupUi(this);
    // this->updateAvailableDevices();
    connect(ui_.push_set_cmd, &QPushButton::clicked, this, &LittlebotGui::sendCommandButtonClicked);
    connect(ui_.push_get_status, &QPushButton::clicked, this, &LittlebotGui::getStatusButtonClicked);
    connect(ui_.push_connect, &QPushButton::clicked, this, &LittlebotGui::hardwareConnection);

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

void LittlebotGui::getStatusButtonClicked()
{
    // Try to construct a LittlebotDriver with the selected device and report result
    int idx = ui_.combo_dev_serial_available->currentIndex();
    if (idx <= 0) {  // 0 is the placeholder "Select a device"
        QMessageBox msgBox(QMessageBox::Warning, "LittleBot", "Please select a device", QMessageBox::Ok, this);
        msgBox.exec();
        return;
    }

    // Our combo has a placeholder at index 0, so device index is (idx - 1)
    auto device_index = static_cast<uint16_t>(idx - 1);
    auto n_ports = available_devices_.scanPorts();
    if (device_index >= n_ports) {
        QMessageBox msgBox(QMessageBox::Warning, "LittleBot", "Selected device index is out of range", QMessageBox::Ok, this);
        msgBox.exec();
        return;
    }

    auto port_path_opt = available_devices_.findPortPath(device_index);
    if (!port_path_opt.has_value()) {
        QMessageBox msgBox(QMessageBox::Critical, "LittleBot", "Could not resolve port path for selected device", QMessageBox::Ok, this);
        msgBox.exec();
        return;
    }

    try {
        auto serial = std::make_shared<littlebot_base::SerialPort>();
        // Try to build the driver and open the port (driver opens in constructor)
        littlebot_base::LittlebotDriver driver(serial, *port_path_opt, 115200);
        QMessageBox msgBox(QMessageBox::Information, "LittleBot", "Driver initialized successfully!", QMessageBox::Ok, this);
        msgBox.exec();
    } catch (const std::exception &ex) {
        QMessageBox msgBox(QMessageBox::Critical, "LittleBot", QString("Driver initialization failed: ") + ex.what(), QMessageBox::Ok, this);
        msgBox.exec();
    }

    emit littlebotStatus();
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

void LittlebotGui::hardwareConnection()
{
    auto serial_port = std::make_shared<littlebot_base::SerialPort>();
    littlebot_driver_ = std::make_shared<littlebot_base::LittlebotDriver>(serial_port, "/dev/rfcomm0", 115200);
}

}  // namespace littlebot_rqt_plugin