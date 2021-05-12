#include "aaw_LDSDriverServer_Locomotory.h"

/* 构造函数。
 * 发布1个ROS Service接受visualServo的交互请求。
 */
AAWLDSServerLocomotory::AAWLDSServerLocomotory(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
    LDSInteractionServer_ = nh_.advertiseService("LDS_Locomotory_Interaction_service", &AAWLDSServerLocomotory::LDSInteractionCallback, this);
    
    LDSPtr_ = new AAWLDSDriverClass("/dev/ttyUSB0");
    showMsg("The locomotory LDS is initialized!");
    turnOffLaser();
}

/* 析构函数，释放动态请求的空间。
 */
AAWLDSServerLocomotory::~AAWLDSServerLocomotory()
{
    delete LDSPtr_;
}

//读取位移
float AAWLDSServerLocomotory::getDistance()
{
    return LDSPtr_->getDistance();
}

//打开激光测距
void AAWLDSServerLocomotory::turnOnLaser()
{
    LDSPtr_->turnOnLaser();
}

//关闭激光测距
void AAWLDSServerLocomotory::turnOffLaser()
{
    LDSPtr_->turnOffLaser();
}

//LDS驱动的回调函数，控制激光开关和位移数据的读取
bool AAWLDSServerLocomotory::LDSInteractionCallback(aaw_ros::LDSInteractionRequest& requestType, aaw_ros::LDSInteractionResponse& response)
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

void AAWLDSServerLocomotory::showMsg(const char * msg)
{
    std::cerr<<msg<<"\n";
}

void AAWLDSServerLocomotory::errorMsg(const char * msg)
{
    std::cerr<<msg<<"\n";
    exit(1);
}

/* 此node实现上方步进电机的远程驱动控制。
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "aaw_LDSDriverServer_Locomotory");
    ros::NodeHandle nh;

    AAWLDSServerLocomotory aldssl(&nh);

    ros::spin();

    return 0;
}
