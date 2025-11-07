#pragma once

#include <string>
#include <memory>

#include <QObject>
#include <QVector>

#include "littlebot_base/littlebot_driver.hpp"

namespace littlebot_rqt_plugin
{
class LittlebotComm : public QObject {
    Q_OBJECT
public:
    explicit LittlebotComm(QObject *parent = nullptr);

public slots:   
    void connectHardware(QString portName);

    void disconnectHardware();

    void receiveVelocitiesCommand(const QVector<float> &data);

signals:
    void sendVelocitiesStatus(const QVector<float> &data);

    void sendPositionsStatus(const QVector<float> &data);

    void errorOccurred(const QString &message);

    void connectionStatus(bool connected);

private:
    std::shared_ptr<littlebot_base::LittlebotDriver> littlebot_driver_;

    std::shared_ptr<littlebot_base::ISerialPort> serial_;

    std::map<std::string, float> command_velocities_{{"left_wheel", 0.0f}, {"right_wheel", 0.0f}};

    std::map<std::string, float> status_positions_{{"left_wheel", 0.0f}, {"right_wheel", 0.0f}};

    std::map<std::string, float> status_velocities_{{"left_wheel", 0.0f}, {"right_wheel", 0.0f}};
};
}  // namespace littlebot_rqt_plugin