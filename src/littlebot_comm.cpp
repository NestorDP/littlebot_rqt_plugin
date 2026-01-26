// @ Copyright 2025-2026 Nestor Neto
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

#include "littlebot_rqt_plugin/littlebot_comm.hpp"

#include "littlebot_base/serial_port.hpp"

namespace littlebot_rqt_plugin
{
LittlebotComm::LittlebotComm(QObject *parent)
: QObject(parent),
  hardware_request_timer_(new QTimer(this)),
  status_update_timer_(new QTimer(this))
{
  connect(hardware_request_timer_, &QTimer::timeout, this,
    [this]() {this->requestStatusFromHardware(false);});
  connect(status_update_timer_, &QTimer::timeout, this,
    [this]() {this->getDataStatus();});
}

LittlebotComm::~LittlebotComm()
{
  this->disconnectHardware();
  if (hardware_request_timer_->isActive()) {
    hardware_request_timer_->stop();
  }
  if (status_update_timer_->isActive()) {
    status_update_timer_->stop();
  }
  
  // Explicitly reset all shared pointers to ensure cleanup
  serial_port_.reset();
  rt_state_buffer_.reset();
  rt_command_buffer_.reset();
  littlebot_driver_.reset();
}

void LittlebotComm::connectHardware(QString portName)
{
  serial_port_ = std::make_shared<littlebot_base::SerialPort>();

  if (!serial_port_->open(portName.toStdString(), 115200)) {
    emit errorOccurred("Failed to open serial port: " + portName);
    return;
  }

  rt_state_buffer_ = std::make_shared<littlebot_base::RosRTBuffer>();
  rt_command_buffer_ = std::make_shared<littlebot_base::RosRTBuffer>();
  std::vector<std::string> joint_names{"left_wheel", "right_wheel"};

  try {
    littlebot_driver_ =
      std::make_shared<littlebot_base::LittlebotDriver>(
        serial_port_,
        rt_state_buffer_,
        rt_command_buffer_,
        joint_names);
  } catch (const std::exception & ex) {
    emit errorOccurred(QString::fromStdString(std::string("Connection failed: ") + ex.what()));
    return;
  }
  if (!hardware_request_timer_->isActive()) {
    this->hardware_request_timer_->start(kRequestTimerInterval_ms);
  }

  emit connectionStatus(true);
}

void LittlebotComm::disconnectHardware()
{
  try {
    if (littlebot_driver_) {
      serial_port_->close();
      littlebot_driver_.reset();
    }
  } catch (const std::exception & ex) {
    emit errorOccurred(
      QString::fromStdString(std::string("Disconnection failed: ") +
        ex.what()));
    return;
  }
    if (hardware_request_timer_->isActive()) {
    this->hardware_request_timer_->stop();
  }
  emit connectionStatus(false);
}

void LittlebotComm::setVelocitiesCommand(const QVector<float> & data)
{
  if (data.size() < 2) {
    emit errorOccurred("Insufficient velocity data received.");
    return;
  }
  // command_velocities_["left_wheel"] = data[0];
  // command_velocities_["right_wheel"] = data[1];
}

void LittlebotComm::startStreamTimer()
{
  if (!status_update_timer_->isActive()) {
    this->status_update_timer_->start(kStatusUpdateTimerInterval_ms);
  }
}

void LittlebotComm::stopStreamTimer()
{
  if (status_update_timer_->isActive()) {
    this->status_update_timer_->stop();
  }
}

void LittlebotComm::requestStatusFromHardware(const bool debug)
{
  auto request_ok = littlebot_driver_->requestStatus();
  if (!request_ok) {
    auto error_code = littlebot_driver_->getLastError();
    emit errorOccurred(QString("Error during status update: %1").arg(static_cast<int>(error_code)));
    this->hardware_request_timer_->stop();
  }

  if (debug) {
    QString message{"Message Test!"};
    emit protocolMessage(message);
  }
}

void LittlebotComm::getDataStatus()
{
  littlebot_base::WheelRTData state;

  if (!littlebot_driver_) {
    emit errorOccurred(QString("Cannot read data status: littlebot_driver_ is not connected."));
    this->stopStreamTimer();
    return;
  }
  
  littlebot_driver_->readRTData(state);

  QVector<float> data;
  data.append(state.status_position[0]);
  data.append(state.status_position[1]);
  data.append(state.status_velocity[0]);
  data.append(state.status_velocity[1]);

  emit dataStatus(data);
}
}  // namespace littlebot_rqt_plugin
