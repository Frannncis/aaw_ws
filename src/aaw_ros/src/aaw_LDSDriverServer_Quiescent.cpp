#include "aaw_LDSDriverServer_Quiescent.h"

/* 构造函数。
 * 发布1个ROS Service接受visualServo的交互请求。
 */
AAWLDSServerQuiescent::AAWLDSServerQuiescent(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
    LDSInteractionServer_ = nh_.advertiseService("LDS_Quiescent_Interaction_service", &AAWLDSServerQuiescent::LDSInteractionCallback, this);
    
    LDSPtr_ = new AAWLDSDriverClass("/dev/ttyUSB0");
    showMsg("The quiescent LDS is initialized!");
    turnOffLaser();
}

/* 析构函数，释放动态请求的空间。
 */
AAWLDSServerQuiescent::~AAWLDSServerQuiescent()
{
    delete LDSPtr_;
}

//读取位移
float AAWLDSServerQuiescent::getDistance()
{
    return LDSPtr_->getDistance();
}

//打开激光测距
void AAWLDSServerQuiescent::turnOnLaser()
{
    LDSPtr_->turnOnLaser();
}

//关闭激光测距
void AAWLDSServerQuiescent::turnOffLaser()
{
    LDSPtr_->turnOffLaser();
}

//LDS驱动的回调函数，控制激光开关和位移数据的读取
bool AAWLDSServerQuiescent::LDSInteractionCallback(aaw_ros::LDSInteractionRequest& requestType, aaw_ros::LDSInteractionResponse& response)
{
    if (requestType.toTurnOnLaser == true) {
        turnOnLaser();
        response.distance = 0;
        response.ExecStatus = 1;
    }
    else if (requestType.toTurnOffLaser == true) {
        turnOffLaser();
        response.distance = 0;
        response.ExecStatus = 1;
    }
    else if (requestType.toReadDistance == true) {
        response.distance = getDistance();
        response.ExecStatus = 1;
    }
    else {
        showMsg("Inappropriate LDS driven request");
        response.distance = 0;
        response.ExecStatus = 0;
    }
    
    return true;
}

void AAWLDSServerQuiescent::showMsg(const char * msg)
{
    std::cerr<<msg<<"\n";
}

void AAWLDSServerQuiescent::errorMsg(const char * msg)
{
    std::cerr<<msg<<"\n";
    exit(1);
}

/* 此node实现上方步进电机的远程驱动控制。
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "aaw_LDSDriverServer_Quiescent");
    ros::NodeHandle nh;

    AAWLDSServerQuiescent aldssq(&nh);

    ros::spin();

    return 0;
}
