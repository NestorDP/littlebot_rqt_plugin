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

#pragma once

#include <QObject>
#include <QTimer>
#include <QVector>

#include <map>
#include <memory>
#include <string>

#include "littlebot_base/littlebot_driver.hpp"
#include "littlebot_base/serial_port.hpp"
#include "littlebot_base/ros_rt_buffer.hpp"
#include "littlebot_base/types.hpp"

namespace littlebot_rqt_plugin
{
class LittlebotComm : public QObject {
  Q_OBJECT

public:
  /**
  * @brief Construct a new Littlebot Comm object
  * 
  * @param parent Parent QObject
  */
  explicit LittlebotComm(QObject *parent = nullptr);

signals:
  /**
   * @brief Signal emitted when new status data is available
   * 
   * @param data Status data
   */
  void dataStatus(const QVector<float> & data);

  /**
   * @brief Signal emitted when an error occurs
   *
   * @param message Protocol message
   */
  void errorOccurred(const QString & message);

  /**
   * @brief Signal emitted to send protocol messages
   *
   * @param message Protocol message
   * 
   * @note This signal is used for debugging purposes
   */
  void protocolMessage(const QString & message);

  /**
   * @brief Signal emitted to indicate connection status
   * 
   * @param connected True if connected, false otherwise
   */
  void connectionStatus(bool connected);

public slots:
  /**
   * @brief Slot to connect to the hardware
   * 
   * @param portName Name of the serial port
   */
  void connectHardware(QString portName);

  /**
   * @brief Slot to disconnect from the hardware
   */
  void disconnectHardware();

  /**
   * @brief Slot to receive velocity commands from GUI
   * 
   * @param data Velocity command data
   */
  void velocitiesCommand(const QVector<float> & data);

  /**
   * @brief Slot to start the data capture timer
   */
  void startTimer();

  /**
   * @brief Slot to stop the data capture timer
   */
  void stopTimer();

  /**
   * @brief Slot to update status data from hardware
   * 
   * @param debug Flag to indicate if debug information is requested
   */
  void requestStatusFromHardware(const bool debug);

private:
  /**
   * @brief Littlebot driver instance
   */
  std::shared_ptr<littlebot_base::LittlebotDriver> littlebot_driver_;

  /**
   * @brief Serial port interface
   */
  std::shared_ptr<littlebot_base::SerialPort> serial_port_;

  /**
   * @brief RT buffer for wheel state data
   */
  std::shared_ptr<littlebot_base::IRTBuffer<littlebot_base::WheelRTData>> rt_state_buffer_;

  /**
   * @brief RT buffer for wheel command data
   */
  std::shared_ptr<littlebot_base::IRTBuffer<littlebot_base::WheelRTData>> rt_command_buffer_;

  /**
   * @brief Command velocities for the wheels
   */
  std::map<std::string, float> command_velocities_{{"left_wheel", 0.0f}, {"right_wheel", 0.0f}};

  /**
   * @brief Status positions for the wheels
   */
  std::map<std::string, float> status_positions_{{"left_wheel", 0.0f}, {"right_wheel", 0.0f}};

  /**
   * @brief Status velocities for the wheels
   */
  std::map<std::string, float> status_velocities_{{"left_wheel", 0.0f}, {"right_wheel", 0.0f}};

  /**
   * @brief Timer for periodic data capture
   */
  QTimer *timer_;

  /**
   * @brief Timer interval in milliseconds
   */
  static constexpr int kTimerInterval_ms{500};
};
}  // namespace littlebot_rqt_plugin
