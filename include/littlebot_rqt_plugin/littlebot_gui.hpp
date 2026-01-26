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

#pragma once

/**
 * To more information about the serial library used,
 * please visit: https://github.com/NestorDP/cppserial
 */
#include <qwt_legend.h>
#include <qwt_plot.h>
#include <qwt_plot_curve.h>

#include <QFile>
#include <QFileDialog>
#include <QMessageBox>
#include <QThread>
#include <QTextStream>
#include <QVector>

#include <memory>
#include <string>
#include <vector>

#include "libserial/device.hpp"
#include "libserial/ports.hpp"

#include "littlebot_rqt_plugin/littlebot_comm.hpp"
#include "ui_littlebot_gui.h" // NOLINT: include order

namespace littlebot_rqt_plugin
{
/**
 * @class LittlebotGui
 * @brief GUI class for the Littlebot RQT plugin
 */
class LittlebotGui : public QDialog
{
  Q_OBJECT

public:
  /**
   * @brief Construct a new Littlebot GUI object
   *
   * @param parent Parent widget
   */
  explicit LittlebotGui(QWidget *parent = nullptr);

  /**
   * @brief Destroy the Littlebot GUI object
   */
  ~LittlebotGui();

  /**
   * @brief Update the status display in the GUI
   *
   * @param data Status data to display
   */
  void updateStatusDisplay(const QVector<float> & data);

  /**
   * @brief Update the list of available serial devices
   */
  void updateAvailableDevices();

  /**
   * @brief Update the curves to plot with new data
   */
  void updateVelocitiesCurves();

  /**
   * @brief Update the setpoint curve to plot with new data
   */
  void updateSetpointCurves();

  /**
   * @brief Update the plots with the latest curves
   */
  void updatePlots();

signals:
  /**
   * @brief Signal emitted when Littlebot status is requested
   */
  void littlebotStatus();

  /**
   * @brief Signal emitted to connect to the hardware
   *
   * @param portName Name of the serial port
   */
  void connectHardware(QString portName);

  /**
   * @brief Signal emitted to disconnect from the hardware
   */
  void disconnectHardware();

  /**
   * @brief Signal emitted to send velocity commands to the Littlebot
   *
   * @param data Velocity command data
   */
  void setVelocitiesCommand(const QVector<float> & data);

    /**
     * @brief Signal emitted to start data capture
     */
  void startStream();

  /**
   * @brief Signal emitted to stop data capture
   */
  void stopStream();

  /**
   * @brief Signal emitted to request data status
   *
   * @param debug Flag to indicate if debug information is requested
   */
  void requestDataStatus(const bool debug);

public slots:
  /**
   * @brief Slot to handle Littlebot command input from ROS
   */
  void littlebotCommand(const std::string & text);

  /**
   * @brief Slot to handle error messages
   *
   * @param message Error message to display
   */
  void showError(const QString & message);

  /**
   * @brief Print a protocol message from hardware abstraction to the GUI
   *
   * @param message Message to print
   */
  void printProtocolMessage(const QString & message);

  /**
   * @brief Update the GUI widgets based on the data status
   *
   * @param data Data status to update the widgets with
   */
  void updateDataStatus(const QVector<float> & data);

  /**
   * @brief Update the GUI widgets based on the connection state
   */
  void updateConnectionState(bool connected);

  /**
   * @brief Update the setpoint based on the GUI input
   */
  void updateSetpoint();

  /**
   * @brief Save the plot data to a file
   */
  void savePlotDataToFile();

private:
    /**
     * @brief Get the status of the Littlebot
     */
    // void getStatus();

    /**
     * @brief Send a command to the Littlebot
     */
    // void setCommand();

  /**
   * @brief UI object for the Littlebot GUI
   */
  Ui::LittlebotGui ui_;

  /**
   * @brief Available serial devices
   */
  libserial::Ports available_devices_;

  /**
   * @brief Selected device index
   */
  uint16_t selected_device_index_{0};

  /**
   * @brief Current number of available devices
   */
  int current_number_of_devices_{0};

  float command_velocity_setpoint_left_{0.0f};

  float command_velocity_setpoint_right_{0.0f};

  std::vector<double> command_velocity_left_{0.0f};

  std::vector<double> command_velocity_right_{0.0f};

  std::vector<double> status_velocity_left_{0.0f};

  std::vector<double> status_velocity_right_{0.0f};

  std::vector<double> status_position_left_{0.0f};

  std::vector<double> status_position_right_{0.0f};

  std::vector<double> plot_index_;

  // Plot data
  QVector<double> plot_x_;
  QVector<double> velocity_left_;
  QVector<double> setpoint_curve_data_;

  // State
  bool connected_{false};
  float setpoint_{0.0f};

  // Plot objects
  QwtPlotCurve *wheel_velocity_curve_{nullptr};
  QwtPlotCurve *setpoint_curve_{nullptr};

  // Constants
  static constexpr int kMaxPoints{500};
  static constexpr float kMinSetpoint{-10.0f};
  static constexpr float kMaxSetpoint{10.0f};


  std::shared_ptr<std::vector<double>> status_velocity_left_ptr_{nullptr};
};

}  // namespace littlebot_rqt_plugin
