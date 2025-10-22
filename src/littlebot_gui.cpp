// @ Copyright 2023-2025 Nestor Neto
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

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
    connect(ui_.push_set_cmd, &QPushButton::clicked, this, &LittlebotGui::sendCommand);
    connect(ui_.push_get_status, &QPushButton::clicked, this, &LittlebotGui::getStatus);
    connect(ui_.push_connect, &QPushButton::clicked, this, &LittlebotGui::connectHardware);

    this->updateAvailableDevices();

    connect(ui_.combo_dev_serial_available, QOverload<int>::of(&QComboBox::activated),
        [this](int index) {this->updateAvailableDevices();
    });
}

void LittlebotGui::sendCommand()
{
    emit littlebotStatus();
}

void LittlebotGui::getStatus()
{
    littlebot_driver_->sendData('S');
    littlebot_driver_->receiveData();
    auto status_positions = littlebot_driver_->getStatusPositions();
    auto status_velocities = littlebot_driver_->getStatusVelocities();

    ui_.lcd_left_pos_status->display(
        QString::number(status_positions["left_wheel"]));
    ui_.lcd_right_pos_status->display(
        QString::number(status_positions["right_wheel"]));

    ui_.lcd_left_vel_status->display(
        QString::number(status_velocities["left_wheel"]));
    ui_.lcd_right_vel_status->display(
        QString::number(status_velocities["right_wheel"]));

    for (const auto& [joint, position] : status_positions) {
        std::cout << "Position - " << joint << ": " << position << std::endl;
    }
    for (const auto& [joint, velocity] : status_velocities) {
        std::cout << "Velocity - " << joint << ": " << velocity << std::endl;
    }

    emit littlebotStatus();
}

void LittlebotGui::updateAvailableDevices()
{
    try{
        auto number_of_devices = available_devices_.scanPorts();
        auto selected_device_id = ui_.combo_dev_serial_available->currentIndex();
        if(selected_device_id < 0) {
            selected_device_id = 0;
        }

        auto port_path_opt = available_devices_.findPortPath(static_cast<uint16_t>(selected_device_id));
        if (port_path_opt.has_value()) {
            ui_.label_device_port->setText(QString::fromStdString(*port_path_opt));
        } else {
            ui_.label_device_port->setText("Unknown");
        }

        if(number_of_devices == current_number_of_devices_) {
            return;
        }
        current_number_of_devices_ = number_of_devices;
        ui_.combo_dev_serial_available->clear();
        std::vector<libserial::Device> devices;
        available_devices_.getDevices(devices);
        for (const auto& device : devices) {
            ui_.combo_dev_serial_available->addItem(QString::fromStdString(device.getName()));
        }
    } catch (const std::exception &ex) {
        current_number_of_devices_ = -1;
        ui_.combo_dev_serial_available->clear();
        ui_.combo_dev_serial_available->addItem("Unknown");
        ui_.label_device_port->setText("Unknown");
        QMessageBox msgBox(QMessageBox::Critical, "LittleBot", QString("Failed to update devices: ") + ex.what(), QMessageBox::Ok, this);
        msgBox.exec();
    }
}

void LittlebotGui::connectHardware()
{
    try {
        auto serial_port = std::make_shared<littlebot_base::SerialPort>();
        auto port_path = ui_.label_device_port->text().toStdString();
        
        littlebot_driver_ = std::make_shared<littlebot_base::LittlebotDriver>(serial_port, port_path, 115200);
    } catch (const std::exception &ex) {
        QMessageBox msgBox(QMessageBox::Critical, "LittleBot", QString("Connection failed: ") + ex.what(), QMessageBox::Ok, this);
        msgBox.exec();
        return;
    }
    QMessageBox msgBox(QMessageBox::Information, "LittleBot", "Connected to device successfully!", QMessageBox::Ok, this);
    msgBox.exec();
}

void LittlebotGui::littlebotCommand(const std::string &text)
{
    // ui_.textBrowser->append(QString::fromStdString(text));
}

}  // namespace littlebot_rqt_plugin