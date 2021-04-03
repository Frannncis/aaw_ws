#ifndef AAWTCPSERVER_H
#define AAWTCPSERVER_H

#include <sys/socket.h>
#include <arpa/inet.h>
#include <iostream>
#include <cstring>
#include <vector>
#include <unistd.h>
#include "jsoncpp/json/json.h"

class AAWTCPServer {
private:
    enum CommandType {ENABLE, DISABLE, MOVE};
    enum {RecvMsgSize_ = 100, SentMsgSize_ = 400};
    static float Velocity_;
    static float Acceleration_;
    static float Deceleration_;
    static float Jerk_;
    static unsigned int ctrlID_;
    static const int reSendTimes_;
    static const unsigned int reSendCoolingSecs_;
    static const int reRecvTimes_;
    static const unsigned int recvTimeOutSecs_;

    unsigned int port_;
    struct sockaddr_in serverAddr_{}, clientAddr_{};
    socklen_t clientAddrLen_{};
    int serverSock_{};
    int clientSock_{};
    char receivedMsg_[RecvMsgSize_]{};
    char sentMsg_[SentMsgSize_]{};
    std::vector<float> velAcc_;
    std::vector<float> pos_;
    struct timeval recvWaitTime_{};

    static void errorMsg(const char * msg);
    static void warningMsg(const char * msg);
    void initAAWTCPServer();
    void setPos(std::vector<float> &pos);
    static void updateCtrlID();
    void convertCtrlValues2JSON(CommandType ct);
    /**
     * 尚未对机器人返回消息的与发送指令的id不一致情况作出处理！
     * @return 返回1代表指令执行成功,0代表指令执行失败.
     */
    int parseReceivedMsg();
    int sendRecv();
    void sendControlMsg();
    void recvRobotFeedback();

    /**
     * 解析阿童木机器人反馈的非标准JSON字符串,改为标准后弃用.
     * @return
     */
    int parseNonStandardJSONRecvMsg();

public:
    AAWTCPServer();
    explicit AAWTCPServer(unsigned int port);
    ~AAWTCPServer();

    void waitUntilConnected();
    int enableRobot();
    int disableRobot();
    void setVelAcc(std::vector<float> &velAcc);
    int move(std::vector<float> &pos);
};

#endif //AAWTCPSERVER_H
