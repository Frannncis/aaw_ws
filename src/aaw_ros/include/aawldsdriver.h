#ifndef AAWLDSDRIVER_H
#define AAWLDSDRIVER_H

#include <ros/ros.h>
#include <iostream>
#include <cstring>
#include <serial/serial.h>

//激光位移传感器的驱动类
class AAWLDSDriverClass
{
public:
    AAWLDSDriverClass(const char * serialPort);
    ~AAWLDSDriverClass();
    void turnOnLaser();
    void turnOffLaser();
    float getDistance();

private:
    serial::Serial serialPort_;

    static const uint8_t STX;
    static const uint8_t ACK;
    static const uint8_t NAK;
    static const uint8_t ETX;
    static const uint8_t cmd_C;
    static const uint8_t cmd_W;
    static const uint8_t cmd_R;

    uint8_t acquireDistance_[6];
    uint8_t laserON_[6];
    uint8_t laserOFF_[6];
    uint8_t readBuffer_[6];
    size_t bytesRead_;

    void setCommand();
    int openSerialPort(const std::string & port);
    void calcBCC(uint8_t command[]);
    void showBytesRead(const uint8_t * buffer, size_t size);

    int isCommandExecuted(const uint8_t * buffer);
    void printErrorCode(const uint8_t * buffer);

    float extractDistance(const uint8_t * buffer);
    
    void errorMsg(const char * msg);
    void showMsg(const char * msg);
};

#endif