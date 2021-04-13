#include "aaw_motorDriverServer_Side.h"

const unsigned int AAWMotorDriverServerSide::motionTime_ = 10; //留给电机的单程运动时间，这个时间之后才允许进行其他动作

/* 构造函数。
 * 发布1个ROS Service接受visualServo的运动控制请求。
 */
AAWMotorDriverServerSide::AAWMotorDriverServerSide(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
    LDSMotionCtrl_ = nh_.advertiseService("LDS_motion_ctrl_service", &AAWMotorDriverServerSide::LDSMotionCtrlCallback, this);

    sideMotorServerPtr_ = new AAWMotorServer(4000);
    ushort vel = 400;   //RPM
    ushort pulses = 9000;

    sideMotorServerPtr_->waitUntilConnected();
    if (sideMotorServerPtr_->setVel(vel))
    {
        if (sideMotorServerPtr_->setPulseCounts(pulses))
        {
            showMsg("Velocity and pulses are set!");
        }
        else
            errorMsg("Pulses setting failed! Please check the TCP connection!");
    }
    else
        errorMsg("Velocity setting failed! Please check the TCP connection!");
    
    showMsg("The side motor is ready for drive control!");
}

/* 析构函数，释放动态请求的空间。
 */
AAWMotorDriverServerSide::~AAWMotorDriverServerSide()
{
    delete sideMotorServerPtr_;
}

/**
 * 将安装在电缸上的LDS推出。
 */
int AAWMotorDriverServerSide::stickOut()
{
    if (sideMotorServerPtr_->stickOut())
    {
        sleep(motionTime_);
        return 1;
    }
    else {
        errorMsg("Motor action failed!");
        return 0;
    }
}

/**
 *将安装在电缸上的LDS收回。
 */
int AAWMotorDriverServerSide::pullBack()
{
    if (sideMotorServerPtr_->pullBack())
    {
        sleep(motionTime_);
        return 1;
    }
    else {
        errorMsg("Motor action failed!");
        return 0;
    }
}

/**
 * 紧急停止电机运动，一般不用
 */
int AAWMotorDriverServerSide::stop()
{
    return sideMotorServerPtr_->stop();
}

void AAWMotorDriverServerSide::showMsg(const char * msg)
{
    std::cerr<<msg<<"\n";
}

void AAWMotorDriverServerSide::errorMsg(const char * msg)
{
    std::cerr<<msg<<"\n";
    exit(1);
}

bool AAWMotorDriverServerSide::LDSMotionCtrlCallback(aaw_ros::LDSMotionCtrlRequest& requestDir, aaw_ros::LDSMotionCtrlResponse& execStatus)
{
    if (requestDir.toPushOut == true)
        execStatus.ExecStatus = stickOut();
    else
        execStatus.ExecStatus = pullBack();
    return true;
}

/* 此node实现侧方步进电机的远程驱动控制。
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "aaw_motorDriverServer_Side");
    ros::NodeHandle nh;

    AAWMotorDriverServerSide amdss(&nh);
    
    ros::spin();

    return 0;
}