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

namespace littlebot_rqt_plugin
{
class LittlebotComm : public QObject {
  Q_OBJECT

public:
  explicit LittlebotComm(QObject *parent = nullptr);

signals:
  void sendDataStatus(const QVector<float> & data);

  void errorOccurred(const QString & message);

  void sendProtocolMessage(const QString & message);

  void connectionStatus(bool connected);

public slots:
  void connectHardware(QString portName);

  void disconnectHardware();

  void receiveVelocitiesCommand(const QVector<float> & data);

  void startTimer();

  void stopTimer();

  void updateStatusDataFromHardware(const bool debug);

private:
  std::shared_ptr<littlebot_base::LittlebotDriver> littlebot_driver_;

  std::shared_ptr<littlebot_base::SerialPort> serial_port_;

  std::shared_ptr<littlebot_base::IRTBuffer<littlebot_base::WheelRTData>> rt_state_buffer_;

  std::shared_ptr<littlebot_base::IRTBuffer<littlebot_base::WheelRTData>> rt_command_buffer_;

  std::map<std::string, float> command_velocities_{{"left_wheel", 0.0f}, {"right_wheel", 0.0f}};

  std::map<std::string, float> status_positions_{{"left_wheel", 0.0f}, {"right_wheel", 0.0f}};

  std::map<std::string, float> status_velocities_{{"left_wheel", 0.0f}, {"right_wheel", 0.0f}};

  QTimer *timer_;

  static constexpr int kTimerInterval_ms{500};
};
}  // namespace littlebot_rqt_plugin
