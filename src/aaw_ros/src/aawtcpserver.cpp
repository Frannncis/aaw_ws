#include "aawtcpserver.h"

float AAWTCPServer::Velocity_ = 5.;
float AAWTCPServer::Acceleration_ = 50.;
float AAWTCPServer::Deceleration_ = 50.;
float AAWTCPServer::Jerk_ = 100.;
unsigned int AAWTCPServer::ctrlID_ = 0;
const int AAWTCPServer::reSendTimes_ = 3;
const unsigned int AAWTCPServer::reSendCoolingSecs_ = 1;
const int AAWTCPServer::reRecvTimes_ = 3;

//超时时间需要设置长一点。如果还在运动就退出recv就会收到失败和正在运动信息，导致重复发送指令，逻辑混乱。
const unsigned int AAWTCPServer::recvTimeOutSecs_ = 30;

//private member functions

/**
 * @brief AAWTCPServer::errorMsg 打印错误消息并退出程序。
 * @param msg 需要打印的消息。
 */
void AAWTCPServer::errorMsg(const char *msg) {
    std::cerr<<msg<<"\n";
    exit(1);
}

/**
 * @brief AAWTCPServer::warningMsg 打印警告消息，但不退出程序。
 * @param msg 需要打印的警告消息。
 */
void AAWTCPServer::warningMsg(const char *msg) {
    std::cerr<<msg<<"\n";
}

/**
 * @brief AAWTCPServer::initAAWTCPServer 供构造函数调用的初始化函数。
 */
void AAWTCPServer::initAAWTCPServer() {
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

    std::cout<<"TCP server started! Port: "<<port_<<"\n";

    //Set default control vel, acc, dec, jerk.
    velAcc_.push_back(Velocity_);
    velAcc_.push_back(Acceleration_);
    velAcc_.push_back(Deceleration_);
    velAcc_.push_back(Jerk_);
}

/**
 * @brief AAWTCPServer::setPos 将用户传入的控制位置信息更新到pos_变量中。
 * @param pos 用户传入的控制位置信息。
 */
void AAWTCPServer::setPos(std::vector<float> &pos) {
    pos_.clear();
    for (int i = 0; i < 6; ++i) {
        pos_.push_back(pos[i]);
    }
}

/**
 * @brief AAWTCPServer::updateCtrlID 更新控制ID，由于并联机器人要求相邻指令的ID号不同，因此每次发消息之前调用该函数更新指令ID号。
 */
void AAWTCPServer::updateCtrlID() {
    if (ctrlID_ >= 100)
        ctrlID_ = 1;
    else
        ctrlID_ += 1;
}

/**
 * @brief AAWTCPServer::convertCtrlValues2JSON 生成符合JSON格式的消息，以备发送。
 * @param ct 控制消息的类型，分为使能、去使能、运动控制，不同类型生成的消息格式不同。
 */
void AAWTCPServer::convertCtrlValues2JSON(AAWTCPServer::CommandType ct) {
    Json::Value root;
    std::string ctrlMsg;
    switch (ct) {
        case ENABLE:
            root["id"] = ctrlID_;
            root["enable"] = true;
            break;
        case DISABLE:
            root["id"] = ctrlID_;
            root["enable"] = false;
            break;
        case MOVE:
            root["id"] = ctrlID_;
            root["x"] = pos_[0];
            root["y"] = pos_[1];
            root["z"] = pos_[2];
            root["a"] = pos_[3];
            root["b"] = pos_[4];
            root["c"] = pos_[5];
            root["vel"] = velAcc_[0];
            root["acc"] = velAcc_[1];
            root["dec"] = velAcc_[2];
            root["jerk"] = velAcc_[3];
            break;
    }
    ctrlMsg = root.toStyledString();
    strcpy(sentMsg_, ctrlMsg.c_str());
}

/**
 * @brief AAWTCPServer::sendControlMsg 使用socket给客户端发送消息。
 * @note 若发送消息失败，服务端会再尝试reSendTimes_次，超次数则退出程序。
 */
void AAWTCPServer::sendControlMsg() {
    int sentTimes = 0;
    while ((send(clientSock_, sentMsg_, SentMsgSize_, 0) == -1) && sentTimes < reSendTimes_) {
        warningMsg("send() error, retrying...");
        updateCtrlID();
        ++sentTimes;
        sleep(reSendCoolingSecs_);
    }

    if (sentTimes >= reSendTimes_) {
        errorMsg("send() error! exiting...");
    }
}

/**
 * @brief AAWTCPServer::recvRobotFeedback 接收客户端回传的消息。
 * @note 若消息接收失败（一般为超时，指令执行失败会有未回传消息导致超时的情况），则服务端尝试重新发送控制指令给客户端，最多reRecvTimes_次，超次数退出程序。
 */
void AAWTCPServer::recvRobotFeedback() {
    int recvTimes = 0;
    while ((recv(clientSock_, receivedMsg_, RecvMsgSize_, 0) == -1) && recvTimes < reRecvTimes_) {
        warningMsg("recv() error, ctrlMsg resending...");
        ++recvTimes;
        updateCtrlID();
        sendControlMsg();
    }

    if (recvTimes >= reRecvTimes_) {
        errorMsg("recv() error, exiting...");
    }
}

/**
 * @brief AAWTCPServer::parseReceivedMsg 解析收到的客户端的消息，判断指令是否成功执行。
 * @note 解析的是标准格式的JSON字符串。
 * @return 返回1则代表指令执行成功，返回0则代表指令执行失败。
 */
int AAWTCPServer::parseReceivedMsg() {
    std::string msg(receivedMsg_);
    Json::Reader reader;
    Json::Value root;
    reader.parse(msg, root);
    if (root["finished"].asInt() == 1)
        return 1;
    else {
        warningMsg("Robot command execution failed!");
        return 0;
    }
}

/**
 * @brief AAWTCPServer::sendRecv 整合发送、接收、解析消息整个过程。
 * @return 返回指令的执行结果。
 */
int AAWTCPServer::sendRecv() {
    sendControlMsg();
//    sleep(1);
    recvRobotFeedback();
    //能走到这里就代表控制指令已经成功发送并收到了反馈.
    //对于收到的反馈究竟是成功还是失败,交给上级处理,AAWTCPServer只负责成功通信.
//    return parseReceivedMsg();
    return parseNonStandardJSONRecvMsg();
}

/**
 * @brief AAWTCPServer::parseNonStandardJSONRecvMsg 代替parseReceivedMsg()解析并联机器人控制器当前返回的非标准JSON字符串，版本更新后会弃用。
 * @return
 */
int AAWTCPServer::parseNonStandardJSONRecvMsg() {
    std::string msg(receivedMsg_);

    if (msg.find("finished:1", 0) != std::string::npos)
        return 1;
    else {
        warningMsg("Robot command execution failed!");
        return 0;
    }
}

//public member functions

AAWTCPServer::AAWTCPServer() : port_(3000){
    initAAWTCPServer();
}

AAWTCPServer::AAWTCPServer(unsigned int port) : port_(port){
    initAAWTCPServer();
}

AAWTCPServer::~AAWTCPServer() {
    close(clientSock_);
    close(serverSock_);
}

/**
 * @brief AAWTCPServer::waitUntilConnected TCP服务端成功开机端口监听后，用户主动调用此函数进行等待客户端连接。
 * @note 由于设置了recv()的超时退出，accept()函数也会进入非阻塞状态，因此设置循环重新恢复accept()的阻塞效果，即在客户端连接成功之前一直阻塞。
 */
void AAWTCPServer::waitUntilConnected() {
    clientAddrLen_ = sizeof(clientAddr_);
    std::cout<<"Waiting for client connection...\n";
    //由于setsockopt设置的超时时间对accept也生效,而又希望accept阻塞,直到有客户端连接上,所以这里循环等待.
    while ((clientSock_ = accept(serverSock_, (sockaddr *)&serverAddr_, &clientAddrLen_)) == -1) ;
    std::cout<<"Client connected!\n";
}

/**
 * @brief AAWTCPServer::setVelAcc 用户调用该函数实现并联机器人的速度、加速度、减速度、加加速度的更新。
 * @note 用户并非必须调用此函数，在初始化函数中设定了默认值。
 * @param velAcc 存有上述4个运动指标的vector。
 */
void AAWTCPServer::setVelAcc(std::vector<float> &velAcc) {
    velAcc_.clear();
    for (int i = 0; i < 4; ++i) {
        velAcc_.push_back(velAcc[i]);
    }
}

/**
 * @brief AAWTCPServer::move 用户调用的实现并联机构运动控制的函数。
 * @param pos 运动控制的目标点，存有x,y,z,a,b,c六个分量的vector。
 * @return 返回指令执行状态，1为执行成功，0为执行失败。
 */
int AAWTCPServer::move(std::vector<float> &pos) {
    setPos(pos);
    updateCtrlID();
    convertCtrlValues2JSON(CommandType::MOVE);
    return sendRecv();
}

/**
 * @brief AAWTCPServer::enableRobot 使能机器人。
 * @return 返回1为使能成功，0为使能失败。
 */
int AAWTCPServer::enableRobot() {
    updateCtrlID();
    convertCtrlValues2JSON(CommandType::ENABLE);
    return sendRecv();
}

/**
 * @brief AAWTCPServer::disableRobot 去使能机器人。
 * @return 返回1为去使能成功，0为去使能失败。
 */
int AAWTCPServer::disableRobot() {
    updateCtrlID();
    convertCtrlValues2JSON(CommandType::DISABLE);
    return sendRecv();
}
