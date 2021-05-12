#include "aawldsdriver.h"

const uint8_t AAWLDSDriverClass::STX = 0x02;
const uint8_t AAWLDSDriverClass::ACK = 0x06;
const uint8_t AAWLDSDriverClass::NAK = 0x15;
const uint8_t AAWLDSDriverClass::ETX = 0x03;
const uint8_t AAWLDSDriverClass::cmd_C = 0x43;
const uint8_t AAWLDSDriverClass::cmd_W = 0x57;
const uint8_t AAWLDSDriverClass::cmd_R = 0x52;

//==========public member functions==========

//激光位移传感器的驱动库
AAWLDSDriverClass::AAWLDSDriverClass(const char * serialPort)
{
    setCommand();
    bytesRead_ = 0;
    // std::cout<<"serial Port: "<<serialPort<<"\n";
    if (!openSerialPort(serialPort))
        errorMsg("Unable to open port!");
}

AAWLDSDriverClass::~AAWLDSDriverClass()
{

}

//打开激光开始测距
void AAWLDSDriverClass::turnOnLaser()
{
    serialPort_.write(laserON_, 6);
    bytesRead_ = serialPort_.read(readBuffer_, 6);
    if (!isCommandExecuted(readBuffer_)) {
        printErrorCode(readBuffer_);
        errorMsg("Failed to turn on laser");
    }
}

//测距完毕关闭激光
void AAWLDSDriverClass::turnOffLaser()
{
    serialPort_.write(laserOFF_, 6);
    bytesRead_ = serialPort_.read(readBuffer_, 6);
    if (!isCommandExecuted(readBuffer_)) {
        printErrorCode(readBuffer_);
        errorMsg("Failed to turn off laser");
    }
}

//读取位移数据，单位为mm
float AAWLDSDriverClass::getDistance()
{
    serialPort_.write(acquireDistance_, 6);
    bytesRead_ = serialPort_.read(readBuffer_, 6);
    // showBytesRead(readBuffer_, bytesRead_);
    return extractDistance(readBuffer_);
}

//==========private member functions==========
//设置常用的指令
void AAWLDSDriverClass::setCommand()
{
    acquireDistance_[0] = STX;
    acquireDistance_[1] = cmd_C;
    acquireDistance_[2] = 0xB0;
    acquireDistance_[3] = 0x01;
    acquireDistance_[4] = ETX;
    calcBCC(acquireDistance_);
    
    laserON_[0] = STX;
    laserON_[1] = cmd_C;
    laserON_[2] = 0xA0;
    laserON_[3] = 0x03;
    laserON_[4] = ETX;
    calcBCC(laserON_);

    laserOFF_[0] = STX;
    laserOFF_[1] = cmd_C;
    laserOFF_[2] = 0xA0;
    laserOFF_[3] = 0x02;
    laserOFF_[4] = ETX;
    calcBCC(laserOFF_);
}

//返回1开启成功，返回0开启失败
int AAWLDSDriverClass::openSerialPort(const std::string & port)
{
    try {
        serialPort_.setPort(port);
        serialPort_.setBaudrate(9600);
        serialPort_.setBytesize(serial::eightbits);
        serialPort_.setParity(serial::parity_none);
        serialPort_.setStopbits(serial::stopbits_one);
        serial::Timeout timeOut_ = serial::Timeout::simpleTimeout(1000);
        serialPort_.setTimeout(timeOut_);
        serialPort_.open();
    }
    catch(serial::IOException & e) {
        std::cerr<<"Exception: "<<e.what()<<std::endl;
        return 0;
    }

    if (serialPort_.isOpen()) {
        showMsg("LDS serial port initialized!");
        return 1;
    }
    else {
        return 0;
    }
}

//计算BCC校验码
void AAWLDSDriverClass::calcBCC(uint8_t command[])
{
    command[5] = command[1]^command[2]^command[3];
}

//以十六进制的方式打印读取到的数据，仅调试用
void AAWLDSDriverClass::showBytesRead(const uint8_t* buffer, size_t size)
{
    for (size_t i = 0; i < size; ++i) {
        std::cout<<std::hex<<+buffer[i]<<" ";
    }
    std::cout<<"\n";
}

//判断发送的指令是否被成功接收并执行，返回１执行成功，返回０执行失败
int AAWLDSDriverClass::isCommandExecuted(const uint8_t * buffer)
{
    switch (buffer[1])
    {
        case ACK:
            return 1;
            break;
        case NAK:
            return 0;
            break;
        default:
            return 0;
    }
}

//若指令执行失败，打印错误代码
void AAWLDSDriverClass::printErrorCode(const uint8_t * buffer)
{
    /* 错误代码表
    02H 地址错误
    04H BCC值错误
    05H 发送的命令不是[C]/[W]/[R]格式的命令
    06H 设定值错误（参数格式错误）
    07H 设定值错误（超出设定值范围）
    */
    std::cout<<"LDS cmd failure, error code: "<<std::hex<<+buffer[2]<<"\n";
}

//解析读取到的位移数据，单位为mm，返回0代表读到的数据有误，可能是超量程了，此传感器能测量的距离范围为50-150mm
float AAWLDSDriverClass::extractDistance(const uint8_t * buffer)
{
    float distance = 0;
    int value = buffer[3]+buffer[2]*256;
    // std::cout<<"Dist value: "<<value<<"\n";
    if (value <= 5000)
        distance = 100 + value/100.;
    else if (value >= 60536)
        distance = 100-(65536-value)/100.;
    else
        distance = 0;
    return distance;
}

void AAWLDSDriverClass::showMsg(const char * msg)
{
    std::cerr<<msg<<"\n";
}

void AAWLDSDriverClass::errorMsg(const char * msg)
{
    std::cerr<<msg<<"\n";
    exit(1);
}