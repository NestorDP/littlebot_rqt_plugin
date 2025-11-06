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
    connect(ui_.push_get_status, &QPushButton::clicked, this, &LittlebotGui::updateStatusDisplay);
    connect(ui_.push_connect, &QPushButton::clicked, this, &LittlebotGui::connectHardware);

    this->updateAvailableDevices();

    connect(ui_.combo_dev_serial_available, QOverload<int>::of(&QComboBox::activated),
        [this](int index) {this->updateAvailableDevices();
    });

    this->updatePlots();
}

void LittlebotGui::sendCommand()
{
    emit littlebotStatus();
}

void LittlebotGui::getStatus()
{
    try{
        if (!littlebot_driver_) {
            throw std::runtime_error("Not connected to hardware");
        }
        littlebot_driver_->sendData('S');
        littlebot_driver_->receiveData();
        status_positions_ = littlebot_driver_->getStatusPositions();
        status_velocities_ = littlebot_driver_->getStatusVelocities();
    } catch (const std::exception &ex) {
        QMessageBox msgBox(QMessageBox::Critical, "LittleBot", QString("Failed to get status: ") + ex.what(), QMessageBox::Ok, this);
        msgBox.exec();
        return;
    }
    // emit littlebotStatus();
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

void LittlebotGui::updateStatusDisplay()
{
    this->getStatus();
    try {
        ui_.lcd_left_pos_status->display(
            QString::number(status_positions_["left_wheel"], 'f', 2));
        ui_.lcd_right_pos_status->display(
            QString::number(status_positions_["right_wheel"], 'f', 2));
        ui_.lcd_left_vel_status->display(
            QString::number(status_velocities_["left_wheel"], 'f', 2));
        ui_.lcd_right_vel_status->display(
            QString::number(status_velocities_["right_wheel"], 'f', 2));
    } catch (const std::exception &ex) {
        QMessageBox msgBox(QMessageBox::Critical, "LittleBot", QString("Failed to update status display: ") + ex.what(), QMessageBox::Ok, this);
        msgBox.exec();
    }
}

void LittlebotGui::updatePlots()
{
    ui_.qwt_plot->setTitle("Simple QwtPlot Update Example");
    ui_.qwt_plot->setAxisTitle(QwtPlot::xBottom, "Index");
    ui_.qwt_plot->setAxisTitle(QwtPlot::yLeft, "Value");

    curve_ = new QwtPlotCurve();
    curve_->attach(ui_.qwt_plot);
    curve_->setPen(Qt::blue, 2);
    curve_->setRenderHint(QwtPlotItem::RenderAntialiased, true);

    // ---- Simple test data ----
    QVector<double> yValues = {1, 2, 3, 4, 5};
    QVector<double> xValues(yValues.size());
    for (int i = 0; i < yValues.size(); ++i)
        xValues[i] = i + 1;

    // ---- Simple for-loop update ----
    QVector<double> currentY;
    QVector<double> currentX;

    for (int i = 0; i < yValues.size(); ++i) {
        currentX.append(xValues[i]);
        currentY.append(yValues[i]);

        curve_->setSamples(currentX, currentY);
        ui_.qwt_plot->replot();

        QThread::msleep(500); // pause for 0.5s so we can see updates
        qApp->processEvents(); // keep GUI responsive during sleep
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