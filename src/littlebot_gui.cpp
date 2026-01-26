// @ Copyright 2023-2026 Nestor Neto
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

#include "littlebot_rqt_plugin/littlebot_gui.hpp"

#include <QMessageBox>

#include <cstdint>

#include <iostream>
#include <stdexcept>

#include "littlebot_base/littlebot_driver.hpp"
#include "littlebot_base/serial_port.hpp"

namespace littlebot_rqt_plugin
{
LittlebotGui::LittlebotGui(QWidget *parent)
: QDialog(parent)
{
  status_velocity_left_ptr_ = std::make_shared<std::vector<double>>(status_velocity_left_);

  ui_.setupUi(this);

  ui_.push_start_capture->setEnabled(false);
  ui_.push_stop_capture->setEnabled(false);

  this->updateAvailableDevices();

  // connect(ui_.push_set_cmd, &QPushButton::clicked, this, &LittlebotGui::sendCommand);


  connect(ui_.line_edit_setpoint, &QLineEdit::editingFinished, this,
    &LittlebotGui::updateSetpoint);
  connect(ui_.push_start_capture, &QPushButton::clicked, this,
    &LittlebotGui::startStream);
  connect(ui_.push_stop_capture, &QPushButton::clicked, this, &LittlebotGui::stopStream);
  connect(ui_.push_get_status, &QPushButton::clicked, this,
    [this]() {emit requestDataStatus(true);});
  connect(ui_.combo_dev_serial_available, QOverload<int>::of(&QComboBox::activated),
    [this](int index) {this->updateAvailableDevices();});
  connect(ui_.push_save, &QPushButton::clicked, this,
    [this](bool) {this->savePlotDataToFile();});

  connect(ui_.push_connect, &QPushButton::clicked, this,
    [this]() {
      QString currentPortName = ui_.label_device_port->text();
      if (connected_) {
        emit disconnectHardware();
        return;
      }
      emit connectHardware(currentPortName);
    });

  ui_.qwt_plot->setTitle("Left Wheel Velocity");

  if (wheel_velocity_curve_ == nullptr) {
    wheel_velocity_curve_ = new QwtPlotCurve();
    wheel_velocity_curve_->attach(ui_.qwt_plot);
  }
  wheel_velocity_curve_->setPen(Qt::blue, 2);
  wheel_velocity_curve_->setRenderHint(QwtPlotItem::RenderAntialiased, true);


  if (setpoint_curve_ == nullptr) {
    setpoint_curve_ = new QwtPlotCurve("Setpoint");
    setpoint_curve_->attach(ui_.qwt_plot);
  }
  setpoint_curve_->setPen(Qt::red, 2);
  setpoint_curve_->setRenderHint(QwtPlotItem::RenderAntialiased, true);

  this->updatePlots();
}

LittlebotGui::~LittlebotGui()
{
  if (wheel_velocity_curve_) {
    wheel_velocity_curve_->detach();
    delete wheel_velocity_curve_;
    wheel_velocity_curve_ = nullptr;
  }

  if (setpoint_curve_) {
    setpoint_curve_->detach();
    delete setpoint_curve_;
    setpoint_curve_ = nullptr;
  }
}

void LittlebotGui::updateAvailableDevices()
{
  try {
    auto number_of_devices = available_devices_.scanPorts();
    auto selected_device_id =
      ui_.combo_dev_serial_available->currentIndex();
    if (selected_device_id < 0) {
      selected_device_id = 0;
    }

    auto port_path_opt =
      available_devices_.findPortPath(static_cast<uint16_t>(
          selected_device_id));
    if (port_path_opt.has_value()) {
      ui_.label_device_port->setText(
          QString::fromStdString(*port_path_opt));
    } else {
      ui_.label_device_port->setText("Unknown");
    }

    if (number_of_devices == current_number_of_devices_) {
      return;
    }
    current_number_of_devices_ = number_of_devices;
    ui_.combo_dev_serial_available->clear();
    std::vector<libserial::Device> devices;
    available_devices_.getDevices(devices);
    for (const auto & device : devices) {
      ui_.combo_dev_serial_available->addItem(
        QString::fromStdString(device.getName()));
    }
  } catch (const std::exception & ex) {
    current_number_of_devices_ = -1;
    ui_.combo_dev_serial_available->clear();
    ui_.combo_dev_serial_available->addItem("Unknown");
    ui_.label_device_port->setText("Unknown");
    QMessageBox msgBox(QMessageBox::Critical, "LittleBot",
      QString("Failed to update devices: ") + ex.what(),
      QMessageBox::Ok, this);
    msgBox.exec();
  }
}

void LittlebotGui::updatePlots()
{
  this->updateVelocitiesCurves();
  this->updateSetpointCurves();
  ui_.qwt_plot->replot();
}

void LittlebotGui::updateVelocitiesCurves()
{
  if (!wheel_velocity_curve_) {
    return;
  }

  wheel_velocity_curve_->setSamples(plot_x_, velocity_left_);
}

void LittlebotGui::updateSetpointCurves()
{
  if (!setpoint_curve_) {
    return;
  }

  setpoint_curve_->setSamples(plot_x_, setpoint_curve_data_);
}

void LittlebotGui::showError(const QString & message)
{
  QMessageBox msgBox(QMessageBox::Critical, "LittleBot", message,
    QMessageBox::Ok, this);
  msgBox.exec();
}

void LittlebotGui::updateConnectionState(bool connected)
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

void LittlebotGui::updateDataStatus(const QVector<float> & data)
{
  if (data.size() < 4) {
    this->showError("Insufficient status data received.");
    return;
  }

  auto position_left = data[0];
  auto position_right = data[1];
  auto velocity_left = data[2];
  auto velocity_right = data[3];

  Q_UNUSED(position_left)
  Q_UNUSED(position_right)
  Q_UNUSED(velocity_right)

  // Append x index
  auto next_x = plot_x_.isEmpty() ? 0.0 : plot_x_.back() + 1.0;
  plot_x_.push_back(next_x);

  // Append Y data
  velocity_left_.push_back(velocity_left);
  setpoint_curve_data_.push_back(setpoint_);

  if (plot_x_.size() > kMaxPoints) {
    plot_x_.removeFirst();
    velocity_left_.removeFirst();
    setpoint_curve_data_.removeFirst();
  }

  if (ui_.tab_widget->currentIndex() == 0) {
    this->updatePlots();
  } else {
    this->updateStatusDisplay(data);
  }
}

void LittlebotGui::littlebotCommand(const std::string & text)
{
  QMessageBox::information(this, "Littlebot Message",
    QString::fromStdString(text));
}

void LittlebotGui::updateSetpoint()
{
  //TODO: Validate input
  bool ok = false;
  float new_setpoint = ui_.line_edit_setpoint->text().toFloat(&ok);
  if (ok) {
    setpoint_ = new_setpoint;
    QVector<float> data;
    
    // Apply the same setpoint to both left and right wheels
    data.append(setpoint_);  // Left wheel velocity
    data.append(setpoint_);  // Right wheel velocity
    emit setVelocitiesCommand(data);
  } else {
    this->showError("Invalid setpoint value.");
    ui_.line_edit_setpoint->setText(QString::number(setpoint_, 'f', 2));
  }
}

void LittlebotGui::savePlotDataToFile()
{
  QString fileName = QFileDialog::getSaveFileName(
    this, "Save Plot Data", "",
    "CSV Files (*.csv);;All Files (*)");
  if (fileName.isEmpty()) {
    return;
  }

  QFile file(fileName);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    this->showError("Could not open file for writing.");
    return;
  }

  QTextStream out(&file);
  out << "Index,Velocity_Left,Velocity_Right,Position_Left," << "Position_Right\n";
  size_t dataSize = plot_index_.size();
  for (size_t i = 0; i < dataSize; ++i) {
    out << plot_index_[i] << "," << status_velocity_left_[i]
        << "," << status_velocity_right_[i] << ","
        << status_position_left_[i] << ","
        << status_position_right_[i] << "\n";
  }

  file.close();
}

void LittlebotGui::updateStatusDisplay(const QVector<float> & data)
{
  try {
    ui_.lcd_left_vel_status->display(QString::number(data[0], 'f', 2));
    ui_.lcd_right_vel_status->display(QString::number(data[1], 'f', 2));
    ui_.lcd_left_pos_status->display(QString::number(data[2], 'f', 2));
    ui_.lcd_right_pos_status->display(QString::number(data[3], 'f', 2));
  } catch (const std::exception & ex) {
    QMessageBox msgBox(QMessageBox::Critical, "LittleBot",
      QString("Failed to update status display: ") +
      ex.what(),
      QMessageBox::Ok, this);
    msgBox.exec();
  }
}

void LittlebotGui::printProtocolMessage(const QString & message)
{
  try {
    ui_.text_browser_protocol->append(message);
  } catch (const std::exception & ex) {
    QMessageBox msgBox(QMessageBox::Critical, "LittleBot",
      QString("Failed to print protocol message: ") +
      ex.what(),
      QMessageBox::Ok, this);
    msgBox.exec();
  }
}

}  // namespace littlebot_rqt_plugin
