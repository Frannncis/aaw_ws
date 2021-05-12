#include "aaw_weightSensorDataReader.h"

AAWWeightSensorReader::AAWWeightSensorReader(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
    weightPub_ = nh_.advertise<aaw_ros::WeightSensorData>("weigt_sensor_data", 1);

    setCommand();
    bytesRead_ = 0;
    if (!openSerialPort("/dev/ttyUSB0"))
        errorMsg("Unable to open port!");
    setFrequency(set12HZ_);
    resetWeightValue2Zero();
}

AAWWeightSensorReader::~AAWWeightSensorReader()
{

}

//在主循环里调用此函数以循环广播力传感器实时数据
void AAWWeightSensorReader::pubWeight()
{
    weight_.weight = getWeight();
    // std::cout<<"Weight sensor data: "<<weight_.weight<<"\n";
    weightPub_.publish(weight_);
}

void AAWWeightSensorReader::setCommand()
{
    acquireWeight_[0] = 0x01;
    acquireWeight_[1] = 0x03;
    acquireWeight_[2] = 0x00;
    acquireWeight_[3] = 0x00;
    acquireWeight_[4] = 0x00;
    acquireWeight_[5] = 0x02;
    acquireWeight_[6] = 0xC4;
    acquireWeight_[7] = 0x0B;
    
    set50HZ_[0] = 0x01;
    set50HZ_[1] = 0x06;
    set50HZ_[2] = 0x00;
    set50HZ_[3] = 0x10;
    set50HZ_[4] = 0x00;
    set50HZ_[5] = 0x01;
    set50HZ_[6] = 0x49;
    set50HZ_[7] = 0xCF;

    set12HZ_[0] = 0x01;
    set12HZ_[1] = 0x06;
    set12HZ_[2] = 0x00;
    set12HZ_[3] = 0x10;
    set12HZ_[4] = 0x00;
    set12HZ_[5] = 0x00;
    set12HZ_[6] = 0x88;
    set12HZ_[7] = 0x0F;

    reset_[0] = 0x01;
    reset_[1] = 0x06;
    reset_[2] = 0x00;
    reset_[3] = 0x11;
    reset_[4] = 0x00;
    reset_[5] = 0x01;
    reset_[6] = 0x18;
    reset_[7] = 0x0F;
}

//返回1开启成功，返回0开启失败
int AAWWeightSensorReader::openSerialPort(const std::string & port)
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
        showMsg("Weight sensor initialized!");
        return 1;
    }
    else {
        return 0;
    }
}

void AAWWeightSensorReader::setFrequency(const uint8_t * sendBuffer)
{
    serialPort_.write(sendBuffer, 8);
    serialPort_.read(readBuffer_, 8);
}

//力传感器去皮操作，即把当前的重量设为0,就是一个自动增量计算，不是校准操作
void AAWWeightSensorReader::resetWeightValue2Zero()
{
    serialPort_.write(reset_, 8);
    serialPort_.read(readBuffer_, 8);
}

int AAWWeightSensorReader::getWeight()
{
    serialPort_.write(acquireWeight_, 8);
    bytesRead_ = serialPort_.read(readBuffer_, 9);
    // showBytesRead(readBuffer_, bytesRead_);
    return extractWeight(readBuffer_);
}

void AAWWeightSensorReader::showBytesRead(const uint8_t* buffer, size_t size)
{
    for (size_t i = 0; i < size; ++i) {
        std::cout<<std::hex<<+buffer[i]<<" ";
    }
    std::cout<<"\n";
}

int AAWWeightSensorReader::extractWeight(const uint8_t * buffer)
{
    int weight = 0;
    weight = buffer[4]+buffer[3]*256+buffer[6]*256*256+buffer[5]*256*256*256;
    return weight;
}

void AAWWeightSensorReader::showMsg(const char * msg)
{
    std::cerr<<msg<<"\n";
}

void AAWWeightSensorReader::errorMsg(const char * msg)
{
    std::cerr<<msg<<"\n";
    exit(1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aaw_weightSensorDataReader");
    ros::NodeHandle nh;

    AAWWeightSensorReader weightSensor_(&nh);
    
    ros::Rate loop_rate(12); //HZ
    while (ros::ok())
    {
        weightSensor_.pubWeight();
        loop_rate.sleep();
    }

    // ros::spin(); //spin是用来处理ros消息接收者的回调函数的，对于只发送消息的node不需要

    return 0;
}
