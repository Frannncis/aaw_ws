#include "aaw_motorDriverServer_Up.h"

const unsigned int AAWMotorDriverServerUp::motionTime_ = 6; //留给电机的单程运动时间，这个时间之后才允许进行其他动作

/* 构造函数。
 * 发布1个ROS Service接受visualServo的运动控制请求。
 */
AAWMotorDriverServerUp::AAWMotorDriverServerUp(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
    boltMotionCtrl_ = nh_.advertiseService("bolt_motion_ctrl_service", &AAWMotorDriverServerUp::boltMotionCtrlCallback, this);

    upMotorServerPtr_ = new AAWMotorServer(4000);
    ushort vel = 200;   //RPM
    ushort pulses = 2100;

    upMotorServerPtr_->waitUntilConnected();
    if (upMotorServerPtr_->setVel(vel))
    {
        if (upMotorServerPtr_->setPulseCounts(pulses))
        {
            showMsg("Velocity and pulses are set!");
        }
        else
            errorMsg("Pulses setting failed! Please check the TCP connection!");
    }
    else
        errorMsg("Velocity setting failed! Please check the TCP connection!");
    
    showMsg("The top motor is ready for drive control!");
}

/* 析构函数，释放动态请求的空间。
 */
AAWMotorDriverServerUp::~AAWMotorDriverServerUp()
{
    delete upMotorServerPtr_;
}

/**
 * 将插销插入孔内
 */
int AAWMotorDriverServerUp::stickOut()
{
    if (upMotorServerPtr_->stickOut())
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
 * 将插销从孔内拔出
 */
int AAWMotorDriverServerUp::pullBack()
{
    if (upMotorServerPtr_->pullBack())
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
int AAWMotorDriverServerUp::stop()
{
    return upMotorServerPtr_->stop();
}

void AAWMotorDriverServerUp::showMsg(const char * msg)
{
    std::cerr<<msg<<"\n";
}

void AAWMotorDriverServerUp::errorMsg(const char * msg)
{
    std::cerr<<msg<<"\n";
    exit(1);
}

bool AAWMotorDriverServerUp::boltMotionCtrlCallback(aaw_ros::BoltMotionCtrlRequest& requestDir, aaw_ros::BoltMotionCtrlResponse& execStatus)
{
    if (requestDir.toInsert == true)
        execStatus.ExecStatus = stickOut();
    else
        execStatus.ExecStatus = pullBack();
    return true;
}

/* 此node实现上方步进电机的远程驱动控制。
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "aaw_motorDriverServer_Up");
    ros::NodeHandle nh;

    AAWMotorDriverServerUp amdsu(&nh);
    
    ros::spin();

    return 0;
}
