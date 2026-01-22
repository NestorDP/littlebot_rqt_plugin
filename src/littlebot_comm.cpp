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
  timer_(new QTimer(this))
{
  connect(timer_, &QTimer::timeout, this,
    [this]() {this->updateStatusFromHardware(false);});
}

void LittlebotComm::connectHardware(QString portName)
{
  // Create a pointer to the serial port
  serial_port_ = std::make_shared<littlebot_base::SerialPort>();

  // Open the serial port and check for errors
  if (!serial_port_->open(portName.toStdString(), 115200)) {
    emit errorOccurred("Failed to open serial port: " + portName);
    return;
  }

  // Create RT buffers for state and command data
  // Create RT buffers for state and command data
  rt_state_buffer_ = std::make_shared<littlebot_base::RosRTBuffer>();
  rt_command_buffer_ = std::make_shared<littlebot_base::RosRTBuffer>();
  std::vector<std::string> joint_names{"left_wheel", "right_wheel"};

  try {
    // Create the Littlebot driver instance and pass the serial port, RT buffers
    // and joint names
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
  emit connectionStatus(false);
}

void LittlebotComm::velocitiesCommand(const QVector<float> & data)
{
  if (data.size() < 2) {
    emit errorOccurred("Insufficient velocity data received.");
    return;
  }
  command_velocities_["left_wheel"] = data[0];
  command_velocities_["right_wheel"] = data[1];
}

void LittlebotComm::startTimer()
{
  if (!timer_->isActive()) {
    timer_->start(kTimerInterval_ms);
  }
}

void LittlebotComm::stopTimer()
{
  if (timer_->isActive()) {
    timer_->stop();
  }
}

void LittlebotComm::updateStatusFromHardware(const bool debug)
{
    // try {
    //     if (littlebot_driver_) {
    //         littlebot_driver_->sendData('S');
    //         if (littlebot_driver_->receiveData() == 'S') {
    //             status_velocities_ = littlebot_driver_->getStatusVelocities();
    //             status_positions_ = littlebot_driver_->getStatusPositions();

    //             QVector<float> data_status{
    //                 status_velocities_["left_wheel"],
    //                 status_velocities_["right_wheel"],
    //                 status_positions_["left_wheel"],
    //                 status_positions_["right_wheel"]
    //             };

    //             if (debug) {
    //                 QString message{"Message Test!"};
    //                 emit protocolMessage(message);
    //             }
    //             emit dataStatus(data_status);
    //         } else {
    //             throw std::runtime_error("Failed to receive status data from hardware.");
    //         }
    //     } else {
    //         throw std::runtime_error("Littlebot driver not initialized.");
    //     }
    // } catch (const std::exception &ex) {
    //     emit errorOccurred(
    //         QString::fromStdString(
    //             std::string("Error during status update: ") +
    //             ex.what()));
    //     this->stopTimer();
    //     return;
    // }
}
}  // namespace littlebot_rqt_plugin
