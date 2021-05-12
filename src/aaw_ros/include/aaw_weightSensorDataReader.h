#ifndef AAW_WEIGHT_SENSOR_DATA_READER_H_
#define AAW_WEIGHT_SENSOR_DATA_READER_H_

#include <ros/ros.h>
#include <iostream>
#include <cstring>
#include <serial/serial.h>
#include <aaw_ros/WeightSensorData.h>

class AAWWeightSensorReader
{
public:
    AAWWeightSensorReader(ros::NodeHandle* nodehandle);
    ~AAWWeightSensorReader();
    void pubWeight();

private:
    ros::NodeHandle nh_;
    ros::Publisher weightPub_;
    aaw_ros::WeightSensorData weight_;

    serial::Serial serialPort_;

    uint8_t acquireWeight_[8];
    uint8_t set50HZ_[8];
    uint8_t set12HZ_[8];
    uint8_t reset_[8];
    uint8_t readBuffer_[8];
    size_t bytesRead_;

    void setCommand();
    int openSerialPort(const std::string & port);
    void setFrequency(const uint8_t * sendBuffer);
    void resetWeightValue2Zero();
    int getWeight();
    void showBytesRead(const uint8_t * buffer, size_t size);
    int extractWeight(const uint8_t * buffer);

    void errorMsg(const char * msg);
    void showMsg(const char * msg);
};

#endif