#ifndef AAWSERVER4MOTOR_H
#define AAWSERVER4MOTOR_H

#include <sys/socket.h>
#include <arpa/inet.h>
#include <iostream>
#include <cstring>
#include <unistd.h>

class AAWMotorServer {
private:

    static const int reSendTimes_;
    static const unsigned int reSendCoolingSecs_;
    static const int reRecvTimes_;
    static const unsigned int recvTimeOutSecs_;

    unsigned int port_;
    struct sockaddr_in serverAddr_{}, clientAddr_{};
    socklen_t clientAddrLen_{};
    int serverSock_{};
    int clientSock_{};

    struct timeval recvWaitTime_{};

    //send msgs
    uint8_t stickOut_[8];
    uint8_t pullBack_[8];
    uint8_t stop_[8];
    uint8_t vel_[8];
    uint8_t pulseCounts_[8];
    uint8_t queryVel_[8];
    uint8_t queryPulseCounts_[8];

    //receive msg
    uint8_t response_[8];

    void initAAWMotorServer();
    void initCommands();

    void calcCRC16Modbus(uint8_t command[]);

    int sendRecv(const uint8_t * sendBuf);
    int sendControlMsg(const uint8_t * buf) const;
    int recvResponse(const uint8_t * sendBuf);
    int parseReceivedMsg(const uint8_t * sendBuf);  //只用于验证写寄存器时返回的数据（成功时返回和发送的数据一样的数据）

    static void errorMsg(const char * msg);
    static void warningMsg(const char * msg);

public:
    //不提供默认构造函数，必须指定要监听的端口号
    explicit AAWMotorServer(unsigned int port);
    ~AAWMotorServer();

    void waitUntilConnected();

    //写寄存器0x06,返回与写入相同的数据.
    int stickOut();
    int pullBack();
    int setVel(ushort speed_RPM);
    int setPulseCounts(ushort outputPulses);
    int stop();
};

#endif //AAWSERVER4MOTOR_H
