#include "aaw_moveRobotServer.h"

/* 构造函数。
 * 发布6个ROS Service接受visualServo的控制请求。
 * 创建与并联机构通讯的TCP server并完成连接，使能机器人并将机器人移动到零点。
 * 主要完成坐标变换与TCP通讯。
 */
AAWMoveRobotServer::AAWMoveRobotServer(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
    //必须先有值，不然直接用[]访问元素会出现内存错误
    ctrlVal_.assign(originalCtrlVal_.begin(), originalCtrlVal_.end());

    moveRobot_camVel_ = nh_.advertiseService("move_robot_input_camVel", &AAWMoveRobotServer::camVelInputCallback, this);
    moveRobot_distanceZ_ = nh_.advertiseService("move_robot_input_distanceZ", &AAWMoveRobotServer::distanceZInputCallback, this);
    moveRobot_ctrlVal_ = nh_.advertiseService("move_robot_input_ctrlVal", &AAWMoveRobotServer::ctrlValInputCallback, this);
    changeRobotStatus_ = nh_.advertiseService("change_robot_status_service", &AAWMoveRobotServer::changeRobotStatusCallback, this);
    changeTimeIntegration_ = nh_.advertiseService("change_time_integration_service", &AAWMoveRobotServer::changeTimeIntegCallback, this);
    updateCoordTransformer_ = nh_.advertiseService("update_coord_trans_service", &AAWMoveRobotServer::updateCoordTransCallback, this);

    myTCPServerPtr_ = new AAWTCPServer(3000);
    std::vector<float> velAcc{5, 10, 10, 10};
    myTCPServerPtr_->setVelAcc(velAcc);
    myTCPServerPtr_->waitUntilConnected();
    sleep(3);
    AAWEnableRobot();

    while(!(myTCPServerPtr_->move(originalCtrlVal_)))
        sleep(1);
    std::cout<<"Moved to original pos, ready to accept visual servo control!\n";
    AAWDisableRobot();

    coordTransformerPtr_ = new AAWCoordTransform(originalCtrlVal_);                    
}

/* 析构函数，释放动态请求的空间。
 */
AAWMoveRobotServer::~AAWMoveRobotServer()
{
    delete myTCPServerPtr_;
    delete coordTransformerPtr_;
}

/* 相机伺服速度输入的运动控制回调函数。
 * 完成坐标变换并驱动控制。
 * @note 一旦开始由相机速度控制的视觉伺服过程，必须一直调用并更新坐标变换类中的当前坐标变换矩阵，即一直调用此处的coordTransformerPtr_->getCtrlVal(camVel)
 * 因为本次伺服依赖于上一次的坐标变换矩阵，不允许在伺服过程中进行其他形式的控制请求（如ctrlVal），这样会扰乱伺服过程。
 */
bool AAWMoveRobotServer::camVelInputCallback(aaw_ros::MoveRobotRequest& requestCamVel, aaw_ros::MoveRobotResponse& execStatus)
{
    Eigen::Matrix<float, 6, 1> camVel;
    camVel(0) = requestCamVel.vx;
    camVel(1) = requestCamVel.vy;
    camVel(2) = requestCamVel.vz;
    camVel(3) = requestCamVel.wx;
    camVel(4) = requestCamVel.wy;
    camVel(5) = requestCamVel.wz;

    ctrlVal_.clear();
    ctrlVal_ = coordTransformerPtr_->getCtrlVal(camVel);
    execStatus.ExecStatus = myTCPServerPtr_->move(ctrlVal_);
    
    return true;
}

/* 要注意在调用这个服务的地方，不能重复发送指令，因为接收的是增量，重复发送就出错了。
 * 或者调用之前就把这个增量拆分开，这样允许多次调用。  
 */
bool AAWMoveRobotServer::distanceZInputCallback(aaw_ros::MoveRobot_DistanceZRequest& requestDistanceZ, aaw_ros::MoveRobot_DistanceZResponse& execStatus)
{
    if (requestDistanceZ.isUp)
    {
        ctrlVal_[2] += requestDistanceZ.goUpDistance;
        execStatus.ExecStatus = myTCPServerPtr_->move(ctrlVal_);
        return true;
    }
    else {
        ctrlVal_[2] -= requestDistanceZ.goDownDistance;
        execStatus.ExecStatus = myTCPServerPtr_->move(ctrlVal_);
        return true;
    }
}

/* 输入并联机构控制量进行运动控制的回调函数，此函数不应在视觉伺服完成之前调用，主要目的是在完成undock之后控制并联机器人回零点。
 */
bool AAWMoveRobotServer::ctrlValInputCallback(aaw_ros::MoveRobot_CtrlValRequest& requestCtrlVal, aaw_ros::MoveRobot_CtrlValResponse& execStatus)
{
    ctrlVal_.clear();
    ctrlVal_[0] = requestCtrlVal.x;
    ctrlVal_[1] = requestCtrlVal.y;
    ctrlVal_[2] = requestCtrlVal.z;
    ctrlVal_[3] = requestCtrlVal.a;
    ctrlVal_[4] = requestCtrlVal.b;
    ctrlVal_[5] = requestCtrlVal.c;
    execStatus.ExecStatus = myTCPServerPtr_->move(ctrlVal_);
    return true;
}

/* 控制并联机器人下使能的回调函数。
 */
bool AAWMoveRobotServer::changeRobotStatusCallback(aaw_ros::ChangeRobotStatusRequest& requestStatus, aaw_ros::ChangeRobotStatusResponse& execStatus)
{
    if (requestStatus.toEnable == true)
        AAWEnableRobot();
    else
        AAWDisableRobot();
    execStatus.ExecStatus = 1;
    return true;
}

/**
 * 更改坐标变换所需时间积分量的回调函数。
 */
bool AAWMoveRobotServer::changeTimeIntegCallback(aaw_ros::ChangeTimeIntegrationRequest& reqTimeInteg, aaw_ros::ChangeTimeIntegrationResponse& execStatus)
{
    coordTransformerPtr_->changeTimeIntegration(reqTimeInteg.TimeIntegration);
    execStatus.ExecStatus = 1;
    return true;
}

/**
 * 更新coordTransformerPtr_指针指向的坐标变换类对象，因为上一次伺服结束之后就没有更新这个类对象的值了，已经失效了。
 */
bool AAWMoveRobotServer::updateCoordTransCallback(aaw_ros::UpdateCoordTransformerRequest& req, aaw_ros::UpdateCoordTransformerResponse& res)
{
    if (req.newTransformer == true) {
        delete coordTransformerPtr_;
        std::vector<float> newCtrlVal{req.x, req.y, req.z, req.a, req.b, req.c};
        coordTransformerPtr_ = new AAWCoordTransform(newCtrlVal);
        res.ExecStatus = 1;
    }
    return true;
}

/* 使能并联机器人,能返回就一定是成功了，否则一直阻塞。
 */
void AAWMoveRobotServer::AAWEnableRobot()
{
    while (!(myTCPServerPtr_->enableRobot()))
        sleep(1);
    sleep(1);   //并联机构使能之后需要等待1秒才能运动
    std::cout<<"Robot enabled!\n";
}

/* 下使能并联机器人,能返回就一定是成功了，否则一直阻塞。
 */
void AAWMoveRobotServer::AAWDisableRobot()
{
    while (!(myTCPServerPtr_->disableRobot()))
        sleep(1);
    std::cout<<"Robot disabled!\n";
}

/* 此node接受运动控制请求，将请求转换成运动控制量，实现并联机构的远程驱动控制。
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "aaw_moveRobotServer");
    ros::NodeHandle nh;

    AAWMoveRobotServer amr(&nh);
    
    ros::spin();

    return 0;
}