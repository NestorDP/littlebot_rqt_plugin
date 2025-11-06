#pragma once

#include <string>
#include <memory>

#include <QObject>
#include <QByteArray>
#include <QVector>

#include "littlebot_base/littlebot_driver.hpp"

namespace littlebot_rqt_plugin
{
class LittlebotComm : public QObject {
    Q_OBJECT
public:
    explicit LittlebotComm(QObject *parent = nullptr);

public slots:
    /**
     * @brief Request data from the Littlebot hardware
     */
    // void dataRequest();
    
    /**
     * @brief Connect to the Littlebot hardware
     */
    void connectHardware(QString portName);

signals:
    // void dataReceived(const QVector<double> &data);
    void errorOccurred(const QString &message);

private:
    /**
     * @brief Shared pointer to the Littlebot driver
     */
    std::shared_ptr<littlebot_base::LittlebotDriver> littlebot_driver_;

    /**
     * @brief Shared pointer to the serial port
     */
    std::shared_ptr<littlebot_base::ISerialPort> serial_;
};
}  // namespace littlebot_rqt_plugin