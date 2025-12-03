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

    ui_.push_start_capture->setEnabled(false);
    ui_.push_stop_capture->setEnabled(false);

    this->updateAvailableDevices();

    // connect(ui_.push_set_cmd, &QPushButton::clicked, this, &LittlebotGui::sendCommand);
    // connect(ui_.push_get_status, &QPushButton::clicked, this, &LittlebotGui::updateStatusDisplay);

    connect(ui_.push_start_capture, &QPushButton::clicked, this, &LittlebotGui::startCapture);
    connect(ui_.push_stop_capture, &QPushButton::clicked, this, &LittlebotGui::stopCapture);

    connect(ui_.combo_dev_serial_available, QOverload<int>::of(&QComboBox::activated),
        [this](int index) {this->updateAvailableDevices();
    });
    connect(ui_.push_connect, &QPushButton::clicked, this, 
    [this]() {
        QString currentPortName = ui_.label_device_port->text();
        if (connected_) {
            emit disconnectHardware();
            return;
        }
        emit connectHardware(currentPortName); 
    });

    this->updatePlots();
}

// void LittlebotGui::setCommand()
// {
//     emit littlebotStatus();
// }

// void LittlebotGui::getStatus()
// {

//     emit littlebotStatus();
// }

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

void LittlebotGui::updatePlots()
{
    ui_.qwt_plot->setTitle("Left Wheel Velocity");
    // ui_.qwt_plot->setAxisTitle(QwtPlot::xBottom, "Index");
    // ui_.qwt_plot->setAxisTitle(QwtPlot::yLeft, "Value");

    // Ensure the curve is created and attached before using it
    if (curve == nullptr) {
        curve = new QwtPlotCurve();
        curve->attach(ui_.qwt_plot);
    }

    auto status_velocity_left_ptr = std::make_shared<std::vector<double>>(status_velocity_left_);

    this->updateCurvesToPlot(status_velocity_left_ptr);

}

void LittlebotGui::updateCurvesToPlot(std::shared_ptr<std::vector<double>> data)
{
    if (!curve) {
        return;
    }

    curve->setPen(Qt::blue, 2);
    curve->setRenderHint(QwtPlotItem::RenderAntialiased, true);

    // Guard against size mismatch
    if (plot_index_.size() != data->size()) {
        // If plot index is empty, generate a simple index
        if (plot_index_.empty()) {
            plot_index_.resize(data->size());
            for (size_t i = 0; i < data->size(); ++i) plot_index_[i] = static_cast<double>(i);
        }
    }

    curve->setSamples(plot_index_.data(), data->data(), std::min(plot_index_.size(), data->size()));

    ui_.qwt_plot->replot();
}


void LittlebotGui::showError(const QString &message)
{
    QMessageBox msgBox(QMessageBox::Critical, "LittleBot", message, QMessageBox::Ok, this);
    msgBox.exec();
}

void LittlebotGui::updateWidgetsWithConnectionState(bool connected)
{
    connected_ = connected;
    if (connected) {
        ui_.push_connect->setText("Disconnect");
        ui_.combo_dev_serial_available->setEnabled(false);
        ui_.label_device_port->setStyleSheet("color: green;");
        ui_.push_start_capture->setEnabled(true);
        ui_.push_stop_capture->setEnabled(true);
    } else {
        ui_.push_connect->setText("Connect");
        ui_.combo_dev_serial_available->setEnabled(true);
        ui_.label_device_port->setStyleSheet("color: black;");
        ui_.push_start_capture->setEnabled(false);
        ui_.push_stop_capture->setEnabled(false);
    }
}
void LittlebotGui::receiveDataStatus(const QVector<float> &data)
{
    if (data.size() < 4) {
        this->showError("Insufficient status data received.");
        return;
    }

    auto velocity_left = data[0];
    auto velocity_right = data[1];
    auto position_left = data[2];
    auto position_right = data[3];

    auto current_index = plot_index_.empty() ? 0 : plot_index_.back() + 1;
    plot_index_.push_back(current_index);

    status_velocity_left_.push_back(velocity_left);
    status_velocity_right_.push_back(velocity_right);
    status_position_left_.push_back(position_left);
    status_position_right_.push_back(position_right);

    if (plot_index_.size() > kMaxPoints) {
        plot_index_.erase(plot_index_.begin());
        status_velocity_left_.erase(status_velocity_left_.begin());
        status_velocity_right_.erase(status_velocity_right_.begin());
        status_position_left_.erase(status_position_left_.begin());
        status_position_right_.erase(status_position_right_.begin());
    }

    this->updatePlots();
}

void LittlebotGui::littlebotCommand(const std::string &text)
{
    QMessageBox::information(this, "Littlebot Message", QString::fromStdString(text));
}

// void LittlebotGui::updateStatusDisplay()
// {
//     this->getStatus();
//     try {
//         ui_.lcd_left_pos_status->display(
//             QString::number(status_positions_["left_wheel"], 'f', 2));
//         ui_.lcd_right_pos_status->display(
//             QString::number(status_positions_["right_wheel"], 'f', 2));
//         ui_.lcd_left_vel_status->display(
//             QString::number(status_velocities_["left_wheel"], 'f', 2));
//         ui_.lcd_right_vel_status->display(
//             QString::number(status_velocities_["right_wheel"], 'f', 2));
//     } catch (const std::exception &ex) {
//         QMessageBox msgBox(QMessageBox::Critical, "LittleBot", QString("Failed to update status display: ") + ex.what(), QMessageBox::Ok, this);
//         msgBox.exec();
//     }
// }

}  // namespace littlebot_rqt_plugin