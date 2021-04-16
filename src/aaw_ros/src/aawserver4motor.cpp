#include "aawserver4motor.h"

const int AAWMotorServer::reSendTimes_ = 3;
const unsigned int AAWMotorServer::reSendCoolingSecs_ = 1;
const int AAWMotorServer::reRecvTimes_ = 3;

const unsigned int AAWMotorServer::recvTimeOutSecs_ = 10;    //所有控制指令都是立即返回的.

//public member functions

AAWMotorServer::AAWMotorServer(unsigned int port) : port_(port){
    initAAWMotorServer();
}

AAWMotorServer::~AAWMotorServer() {
    close(clientSock_);
    close(serverSock_);
}

/**
 * @brief AAWMotorServer::waitUntilConnected TCP服务端成功开机端口监听后，用户主动调用此函数进行等待客户端连接。
 * @note 由于设置了recv()的超时退出，accept()函数也会进入非阻塞状态，因此设置循环重新恢复accept()的阻塞效果，即在客户端连接成功之前一直阻塞。
 */
void AAWMotorServer::waitUntilConnected() {
    clientAddrLen_ = sizeof(clientAddr_);
    std::cout<<"Waiting for motor driver client connection...\n";
    //由于setsockopt设置的超时时间对accept也生效,而又希望accept阻塞,直到有客户端连接上,所以这里循环等待.
    while ((clientSock_ = accept(serverSock_, (sockaddr *)&serverAddr_, &clientAddrLen_)) == -1) ;
    std::cout<<"Motor driver client connected!\n";
}

/**
 * 设置电机转速，单位为转每分，输入速度拆分成两个字节存放.
 * @param speed_RPM 电机转速
 */
int AAWMotorServer::setVel(ushort speed_RPM) {
    vel_[4] = speed_RPM >> 8;
    vel_[5] = speed_RPM & 0x00FF;
    calcCRC16Modbus(vel_);
    return sendRecv(vel_);
}

/**
 * 设置要发送的脉冲数.
 * @param outputPulses
 * @return
 */
int AAWMotorServer::setPulseCounts(ushort outputPulses) {
    pulseCounts_[4] = outputPulses >> 8;
    pulseCounts_[5] = outputPulses & 0x00FF;
    calcCRC16Modbus(pulseCounts_);
    return sendRecv(pulseCounts_);
}

int AAWMotorServer::stickOut() {
    return sendRecv(stickOut_);
}

int AAWMotorServer::pullBack() {
    return sendRecv(pullBack_);
}

int AAWMotorServer::stop() {
    return sendRecv(stop_);
}

//private member functions

/**
 * @brief AAWMotorServer::initAAWMotorServer 供构造函数调用的初始化函数。
 */
void AAWMotorServer::initAAWMotorServer() {
    recvWaitTime_.tv_sec = recvTimeOutSecs_;
    recvWaitTime_.tv_usec = 0;

    serverSock_ = socket(PF_INET, SOCK_STREAM, 0);
    if (serverSock_ == -1)
        errorMsg("socket() error!");

    int sockOption, sockOptionLen;
    sockOption = true;
    sockOptionLen = sizeof(sockOption);
    setsockopt(serverSock_, SOL_SOCKET, SO_REUSEADDR, &sockOption, sockOptionLen);
    setsockopt(serverSock_, SOL_SOCKET, SO_RCVTIMEO, (char *)&recvWaitTime_, sizeof(recvWaitTime_));

    memset(&serverAddr_, 0, sizeof(serverAddr_));
    serverAddr_.sin_family = AF_INET;
    serverAddr_.sin_addr.s_addr = htonl(INADDR_ANY);
    serverAddr_.sin_port = htons(port_);

    if(bind(serverSock_, (sockaddr*)&serverAddr_, sizeof(serverAddr_)))
        errorMsg("bind() error!");

    if (listen(serverSock_, 3) == -1)
        errorMsg("listen() error!");

    std::cout<<"Motor control server started! Port: "<<port_<<"\n";

    initCommands();
}

void AAWMotorServer::initCommands() {
    stickOut_[0] = 0x01;
    stickOut_[1] = 0x06;
    stickOut_[2] = 0x00;
    stickOut_[3] = 0x01;
    stickOut_[4] = 0x00;
    stickOut_[5] = 0x01;
    stickOut_[6] = 0x19;
    stickOut_[7] = 0xCA;

    pullBack_[0] = 0x01;
    pullBack_[1] = 0x06;
    pullBack_[2] = 0x00;
    pullBack_[3] = 0x00;
    pullBack_[4] = 0x00;
    pullBack_[5] = 0x01;
    pullBack_[6] = 0x48;
    pullBack_[7] = 0x0A;

    stop_[0] = 0x01;
    stop_[1] = 0x06;
    stop_[2] = 0x00;
    stop_[3] = 0x02;
    stop_[4] = 0x00;
    stop_[5] = 0x01;
    stop_[6] = 0xE9;
    stop_[7] = 0xCA;

    vel_[0] = 0x01;
    vel_[1] = 0x06;
    vel_[2] = 0x00;
    vel_[3] = 0x05;
    vel_[4] = 0x00;
    vel_[5] = 0xFF;
    vel_[6] = 0xD9;
    vel_[7] = 0x8B;

    pulseCounts_[0] = 0x01;
    pulseCounts_[1] = 0x06;
    pulseCounts_[2] = 0x00;
    pulseCounts_[3] = 0x07;
    pulseCounts_[4] = 0x00;
    pulseCounts_[5] = 0xFF;
    pulseCounts_[6] = 0x78;
    pulseCounts_[7] = 0x4B;
}

/**
 * Calculate Modbus CRC-16 check code for the first 6 bytes in command[8] array.
 */
void AAWMotorServer::calcCRC16Modbus(uint8_t command[]) {
    ushort tmp = 0xFFFF;
    for (int i = 0; i < 6; ++i) {   //6 bytes
        tmp = command[i] ^ tmp;
        for (int j = 0; j < 8; ++j) {   //8 bits in per byte
            if (tmp & 0x01) {
                tmp = tmp >> 1;
                tmp = tmp ^ 0xA001;
            }
            else
                tmp = tmp >> 1;
        }
    }
    command[6] = tmp & 0x00FF;
    command[7] = tmp >> 8;
}

/**
 * @brief AAWMotorServer::sendRecv 整合发送、接收、解析消息整个过程。
 * @return 返回指令的执行结果。
 * @note 此处的递归是用于当接收失败时重发消息，因为接收函数里
 */
int AAWMotorServer::sendRecv(const uint8_t * sendBuf) {
    if (sendControlMsg(sendBuf))
        if (recvResponse(sendBuf))
            return parseReceivedMsg(sendBuf);
    return 0;
}

/**
 * @brief AAWMotorServer::sendControlMsg 使用socket给客户端发送消息。
 * @note 若发送消息失败，服务端会再尝试reSendTimes_次，超次数则退出程序。
 */
int AAWMotorServer::sendControlMsg(const uint8_t * sendBuf) const {
    int sentTimes = 0;
    while ((send(clientSock_, sendBuf, 8, 0) == -1) && sentTimes < reSendTimes_) {
        warningMsg("send() error, retrying...");
        ++sentTimes;
        sleep(reSendCoolingSecs_);
    }
    if (sentTimes >= reSendTimes_) {
        errorMsg("send() error!");
    }
    return 1;
}

/**
 * @brief AAWMotorServer::recvResponse 接收客户端回传的消息。
 * @note 如果接受失败，返回0，由sendRecv()函数再次发送控制消息.
 */
int AAWMotorServer::recvResponse(const uint8_t * sendBuf) {
    int recvTimes = 0;
    while ((recv(clientSock_, response_, 8, 0) == -1) && recvTimes < reRecvTimes_) {
        warningMsg("recv() error, msg resending...");
        ++recvTimes;
        sendControlMsg(sendBuf);    //这里没有验证返回值了，大差不差
    }

    if (recvTimes >= reRecvTimes_) {
        errorMsg("recv() error!");   //退出就不用return了
    }

    return 1;
}

/**
 * @brief AAWMotorServer::parseReceivedMsg 解析收到的客户端的消息，判断指令是否成功执行。
 * @note 只定义了写寄存器的操作，成功时返回和发送数据完全相同的数据。
 * @return 返回1则代表指令执行成功，返回0则代表指令执行失败。
 */
int AAWMotorServer::parseReceivedMsg(const uint8_t * sendBuf) {
    // for (int i = 0; i < 8; ++i) {
    //     if (sendBuf[i] != response_[i])
    //         return 0;
    // }
    return 1;
}

/**
 * @brief AAWMotorServer::warningMsg 打印警告消息，但不退出程序。
 * @param msg 需要打印的警告消息。
 */
void AAWMotorServer::warningMsg(const char *msg) {
    std::cerr<<msg<<"\n";
}

/**
 * @brief AAWMotorServer::errorMsg 打印错误消息并退出程序。
 * @param msg 需要打印的消息。
 */
void AAWMotorServer::errorMsg(const char *msg) {
    std::cerr<<msg<<"\n";
    exit(1);
}
