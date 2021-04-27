#include "aaw_visualServo.h"

namespace visualServo {
    /* 相机图像话题的回调函数，是本node的主循环结构。
     * 主要完成图像特征提取与视觉伺服。
     * 在视觉伺服完成之前进行伺服控制，完成之后进行对接、等待、分离、下使能机器人等操作。
     */
    void imageCb(const sensor_msgs::ImageConstPtr& leftImage, const sensor_msgs::ImageConstPtr& rightImage) {
        if (taskFinished_)
            return;

        if (readyToGoHome_) {
            withdrawAndGoHome();
            if (toDock_)
                showMsg("Docking completed!");
            else
                showMsg("Undocking completed!");
            taskFinished_ = true;
            disableRobot();
            // waitAndWakeUpAction();
            // while (!askCarToMove()) //执行结束，让小车移动
            //     sleep(1);
            return;
        }

        cv_bridge::CvImagePtr cv_ptr_left, cv_ptr_right;
        try {
            cv_ptr_left = cv_bridge::toCvCopy(leftImage, sensor_msgs::image_encodings::BGR8);
            cv_ptr_right = cv_bridge::toCvCopy(rightImage, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat grayImageLeft, grayImageRight;
        cv::cvtColor(cv_ptr_left->image, grayImageLeft, cv::COLOR_BGR2GRAY);
        cv::cvtColor(cv_ptr_right->image, grayImageRight, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(grayImageLeft, grayImageLeft, cv::Size(3,3),0,0);
        cv::GaussianBlur(grayImageRight, grayImageRight, cv::Size(3,3),0,0);

        AAWVertexesGainer2 vg4Left, vg4Right;
        vg4Left = AAWVertexesGainer2(grayImageLeft);
        vg4Right = AAWVertexesGainer2(grayImageRight);

        cv::imshow(Left_View, grayImageLeft);
        cv::imshow(Right_View, grayImageRight);

        //出现这种情况是因为相机与物体靠得太近，需要由并联机构进行位置调整，车前后动是解决不了的
        //每次调整都需要updateCoordTrans(std::vector<float> & robotCtrlVal)传去并联机构位置调整后的控制量
        if (vg4Right.isLeftBoundOutaView() && vg4Left.isRightBoundOutaView()) {
            ROS_WARN("Camera too close to main body!");
            newCtrlVal_[1] -= cameraStepBackLength_;    //并联机构沿其y轴方向后退

            ensureMove2Pos(newCtrlVal_);

            updateCoordTrans(newCtrlVal_);
            return;
        }
        //否则的话就是有一侧出视野，由小车前后动就可以完成，只要不动到并联机构，就不需要调用updateCoordTrans()
        else {
            if (vg4Right.isLeftBoundOutaView() || vg4Left.isLeftBoundOutaView()) {
                ROS_WARN("Left of main body out of view!");
                while (!carCreepingForward())
                    sleep(1);
                sleep(1);   //发给小车的请求是立即返回的，所以需要等待小车动作结束，再进行图像读取
                return;
            }
            if (vg4Left.isRightBoundOutaView() || vg4Right.isRightBoundOutaView()) {
                ROS_WARN("Right of main body out of view!");
                while (!carCreepingBackward())
                    sleep(1);
                sleep(1);
                return;
            }
        }
        
        ibvsPtr->updateVertexesCoordinates(vg4Left.get4Vertexes(), vg4Right.get4Vertexes());
        ibvsPtr->updateControlLaw();

        if (ibvsPtr->isDesiredPosArrived()) {
            // cv::destroyWindow(Left_View);
            // cv::destroyWindow(Right_View);

            //dock
            ROS_WARN("Desired pos arrived, now trying to dock, watch out!");
            ensureDock();
            
            //到这里就代表并联机构已经托住弹体了，接下来区分是插入插销还是拔出插销了
            if (toDock_) {
                //插入
                if (insertBolt()) {
                    showMsg("Docking completed!");
                    readyToGoHome_ = true;
                    return;
                }
                else {
                    errorMsg("Bolt insertion failed!");
                }
            }
            else {
                if (pullOutBolt()) {
                    showMsg("Undocking completed!");
                    readyToGoHome_ = true;
                    return;
                }
                else {
                    errorMsg("Bolt pulling out failed!");
                }
            }
        }
        else {
            checkAndChangeTimeIntegration();

            //visual servo 一旦开始伺服，到伺服结束之前，不允许进行其他形式的伺服控制请求（如ctrlVal）
            Eigen::Matrix<float, 6, 1> cameraVel;
            cameraVel = ibvsPtr->getCamCtrlVel();
            std::cout<<"Control velocity:\n"<<cameraVel<<std::endl;

            // cv::imshow(Left_View, grayImageLeft);
            // cv::imshow(Right_View, grayImageRight);

            ensureVisualServoRobot(cameraVel);
        }
        
        cv::waitKey(3);
    }

    //重启并联机器人动作
    void wakeUpAction()
    {
        toDock_ = !toDock_; //反转上方插销动作,其他的过程都一样，更新一下初始值即可。
        readyToGoHome_ = false;
        taskFinished_ = false;
        timeIntegrationChanged_ = false;
        enableRobot();
        updateCoordTrans(originalCtrlVal_);
        newCtrlVal_.clear();
        newCtrlVal_.assign(originalCtrlVal_.begin(), originalCtrlVal_.end());
    }

    /**
     * 请求更新坐标变换类对象，每一次使用ensureMove2Pos()驱动并联机器人，并要开始伺服之前都需要调用此函数
     */
    int updateCoordTrans(std::vector<float> & robotCtrlVal)
    {
        aaw_ros::UpdateCoordTransformer updateCoordTransSrv;
        updateCoordTransSrv.request.newTransformer = true;
        updateCoordTransSrv.request.x = robotCtrlVal[0];
        updateCoordTransSrv.request.y = robotCtrlVal[1];
        updateCoordTransSrv.request.z = robotCtrlVal[2];
        updateCoordTransSrv.request.a = robotCtrlVal[3];
        updateCoordTransSrv.request.b = robotCtrlVal[4];
        updateCoordTransSrv.request.c = robotCtrlVal[5];

        unsigned int commuRetryingCount_ = 0;

        while ((commuRetryingCount_ < communicationRetryingTimes_) && !(updateCoordTransClientPtr->call(updateCoordTransSrv))) {
            ++commuRetryingCount_;
            showMsg("Retrying to call updateCoordTrans service...");
            sleep(1);
        }
        if (commuRetryingCount_ >= communicationRetryingTimes_)
            errorMsg("Fail to call updateCoordTrans service!");
        else
            return 1;   //只要能返回，就是成功的.
    }

    void waitAndWakeUpAction()
    {
        sleep(5);
        wakeUpAction();
    }

    bool restartRobotMotionCallback(interaction::RestartRobotMotionRequest& requestMotion, interaction::RestartRobotMotionResponse& execStatus)
    {
        if (requestMotion.ToMove == true) {
            sleep(3);
            wakeUpAction();
            execStatus.ExecStatus = 1;
        }
        else
            execStatus.ExecStatus = 0;
        return true;
    }

    //接收并处理力传感器的信息
    void weightSensorDataCallback(const aaw_ros::WeightSensorData::ConstPtr & msg)
    {
        if (msg->weight >= outaSecurityWeight_)
            std::cerr<<"Weight is "<<msg->weight<<", overloaded!\n";
        withdrawAndRestartServo();
    }

    //获取并联机构当前的控制量
    void robotCtrlValCallback(const aaw_ros::CurrentRobotCtrlVal::ConstPtr & msg)
    {
        currentCtrlVal_[0] = msg->x;
        currentCtrlVal_[1] = msg->y;
        currentCtrlVal_[2] = msg->z;
        currentCtrlVal_[3] = msg->a;
        currentCtrlVal_[4] = msg->b;
        currentCtrlVal_[5] = msg->c;
    }

    //当力传感器接收到异常数据时，安全撤回并联机构并重新启动伺服
    void withdrawAndRestartServo()
    {
        ensureWithdraw(currentCtrlVal_[2]-originalCtrlVal_[2]);
        ensureMove2Pos(originalCtrlVal_);
        wakeUpAction();
        toDock_ = !toDock_; //把wakeUpAction()里反转的状态反转回来
    }

    //完成一次对接或者分离动作，给小车发送运动指令
    int askCarToMove()
    {
        sleep(3);
        interaction::MoveCar moveCarSrv;
        moveCarSrv.request.ToMove = true;

        unsigned int commuRetryingCount_ = 0;
        while ((commuRetryingCount_ < communicationRetryingTimes_) && !(moveCarClientPtr->call(moveCarSrv))) {
            ++commuRetryingCount_;
            showMsg("Retrying to call MoveCar service...");
            sleep(1);
        }
        if (commuRetryingCount_ >= communicationRetryingTimes_)
            errorMsg("Fail to call MoveCar service!");
        else
            return moveCarSrv.response.ExecStatus;
    }

    //向前微调小车位置
    int carCreepingForward()
    {
        interaction::AdjustCarPos adjustCarPosSrv;
        adjustCarPosSrv.request.MoveForward = true;

        unsigned int commuRetryingCount_ = 0;
        while ((commuRetryingCount_ < communicationRetryingTimes_) && !(adjustCarPosClientPtr->call(adjustCarPosSrv))) {
            ++commuRetryingCount_;
            showMsg("Retrying to call AdjustCarPos service...");
            sleep(1);
        }
        if (commuRetryingCount_ >= communicationRetryingTimes_)
            errorMsg("Fail to call AdjustCarPos service!");
        else
            return adjustCarPosSrv.response.ExecStatus;
    }

    //向后微调小车位置
    int carCreepingBackward()
    {
        interaction::AdjustCarPos adjustCarPosSrv;
        adjustCarPosSrv.request.MoveForward = false;

        unsigned int commuRetryingCount_ = 0;
        while ((commuRetryingCount_ < communicationRetryingTimes_) && !(adjustCarPosClientPtr->call(adjustCarPosSrv))) {
            ++commuRetryingCount_;
            showMsg("Retrying to call AdjustCarPos service...");
            sleep(1);
        }
        if (commuRetryingCount_ >= communicationRetryingTimes_)
            errorMsg("Fail to call AdjustCarPos service!");
        else
            return adjustCarPosSrv.response.ExecStatus;
    }

    /* dock()暂时只能尝试一次，不允许多次请求，因为不知道并联机构执行到了什么位置，靠的是增量计算，多次增量会造成安全事故。
     * 其他控制请求函数允许多次请求。
     * 当未唤起moveRobotServer中的服务时，允许重复尝试。
     */
    int dock()
    {
        aaw_ros::MoveRobot_DistanceZ moveDistanceSrv;
        moveDistanceSrv.request.isUp = true;
        moveDistanceSrv.request.goUpDistance = moveUpDistance_;
        if (moveClientPtr_DistanceZ->call(moveDistanceSrv)) {
            ROS_INFO("Feedback from server: %d", moveDistanceSrv.response.ExecStatus);
            if (moveDistanceSrv.response.ExecStatus == 1) {
                return 1;
            }
            else {
                errorMsg("Something terrible happend, docking failed, exiting...");
            }
        }
        else {
            ROS_ERROR("Failed to call service \"move_robot_input_distanceZ\"");
            return 0;   //当出现这种服务没唤起的情况，允许重复尝试唤起。
        }
    }

    void ensureDock()
    {
        unsigned int commuRetryingCount_ = 0;
        while((commuRetryingCount_ < communicationRetryingTimes_) && !dock()) {
            ++commuRetryingCount_;
            showMsg("Retrying to call moveRobotServer...");
            sleep(1);
        }
        if (commuRetryingCount_ >= communicationRetryingTimes_) {
            errorMsg("Communication with service \"move_robot_input_distanceZ\" failed!");
        }
    }

    /* 视觉伺服速度控制请求函数，执行失败时允许重复请求。
     */
    int visualServoRobot(Eigen::Matrix<float, 6, 1> & camCtrlVel)
    {
        aaw_ros::MoveRobot moveSrv;
        moveSrv.request.vx = camCtrlVel(0);
        moveSrv.request.vy = camCtrlVel(1);
        moveSrv.request.vz = camCtrlVel(2);
        moveSrv.request.wx = camCtrlVel(3);
        moveSrv.request.wy = camCtrlVel(4);
        moveSrv.request.wz = camCtrlVel(5);

        if (moveClientPtr->call(moveSrv)) {
            ROS_INFO("Feedback from server: %d", moveSrv.response.ExecStatus);
            return moveSrv.response.ExecStatus;
        }
        else {
            ROS_ERROR("Failed to call service \"move_robot_input_camVel\"");
            return 0;
        }
    }

    void ensureVisualServoRobot(Eigen::Matrix<float, 6, 1> & camCtrlVel)
    {
        unsigned int commuRetryingCount_ = 0;
        while ((commuRetryingCount_ < communicationRetryingTimes_) && !visualServoRobot(camCtrlVel)) {
            ++commuRetryingCount_;
            showMsg("Retrying visual servo...");
            sleep(1);
        }
        if (commuRetryingCount_ >= communicationRetryingTimes_) {
            errorMsg("Visual servo error!");
        }
    }

    /* 实现并联机构回撤、回零点控制。当上方的插销动作完成之后就调用该函数进行处理。
     */
    void withdrawAndGoHome()
    {
        //事实上这里处理的不仅仅是通信失败的情况，还有并联机构收到指令但执行失败的情况
        //trying to withdraw
        ROS_WARN("Now trying to withdraw, watch out!");
        ensureWithdraw(moveDownDistance_);

        //trying to go home
        showMsg("Now trying to go home...");
        ensureMove2Pos(originalCtrlVal_);
        showMsg("Going home completed!");
    }

    /*　并联机构向下收回动作。
     */
    int withdraw(float distance)
    {
        aaw_ros::MoveRobot_DistanceZ moveDistanceSrv;
        moveDistanceSrv.request.isUp = false;
        moveDistanceSrv.request.goDownDistance = distance;
        if (moveClientPtr_DistanceZ->call(moveDistanceSrv)) {
            ROS_INFO("Feedback from server: %d", moveDistanceSrv.response.ExecStatus);
            return moveDistanceSrv.response.ExecStatus;
        }
        else {
            ROS_ERROR("Failed to call service \"move_robot_input_distanceZ\"");
            return 0;
        }
    }

    //能返回就一定成功执行了指令
    void ensureWithdraw(float distance)
    {
        unsigned int commuRetryingCount_ = 0;
        while ((commuRetryingCount_ < communicationRetryingTimes_) && !withdraw(distance)) {
            ++commuRetryingCount_;
            showMsg("Retrying to withdraw...");
            sleep(1);
        }
        if (commuRetryingCount_ >= communicationRetryingTimes_) {
            errorMsg("Withdraw failed!");
        }
    }

    /* 回零点控制。
     */
    int move2Pos(std::vector<float> & robotCtrlVal)
    {
        aaw_ros::MoveRobot_CtrlVal moveCtrlValSrv;
        moveCtrlValSrv.request.x = robotCtrlVal[0];
        moveCtrlValSrv.request.y = robotCtrlVal[1];
        moveCtrlValSrv.request.z = robotCtrlVal[2];
        moveCtrlValSrv.request.a = robotCtrlVal[3];
        moveCtrlValSrv.request.b = robotCtrlVal[4];
        moveCtrlValSrv.request.c = robotCtrlVal[5];

        if (moveClientPtr_CtrlVal->call(moveCtrlValSrv)) {
            ROS_INFO("Feedback from server: %d", moveCtrlValSrv.response.ExecStatus);
            return moveCtrlValSrv.response.ExecStatus;
        }
        else {
            ROS_ERROR("Failed to call service \"move_robot_input_ctrlVal\"");
            return 0;
        }
    }

    //能返回就一定成功执行了指令
    void ensureMove2Pos(std::vector<float> & robotCtrlVal)
    {
        unsigned int commuRetryingCount_ = 0;
        while ((commuRetryingCount_ < communicationRetryingTimes_) && !move2Pos(robotCtrlVal)) {
            ++commuRetryingCount_;
            showMsg("Retrying to move to pos...");
            sleep(1);
        }
        if (commuRetryingCount_ >= communicationRetryingTimes_) {
            errorMsg("Move to pos failed!");
        }
    }

    /* 使能机器人控制函数
     */
    void enableRobot()
    {
        aaw_ros::ChangeRobotStatus changeRobotStatusSrv;
        changeRobotStatusSrv.request.toEnable = true;

        unsigned int commuRetryingCount_ = 0;

        while ((commuRetryingCount_ < communicationRetryingTimes_) && !(changeRobotStatusClientPtr->call(changeRobotStatusSrv))) {
            ++commuRetryingCount_;
            showMsg("Retrying to call changeRobotStatus service...");
            sleep(1);
        }
        if (commuRetryingCount_ >= communicationRetryingTimes_)
            errorMsg("Fail to call changeRobotStatus service!");
    }

    /* 下使能机器人控制函数
     */
    void disableRobot()
    {
        aaw_ros::ChangeRobotStatus changeRobotStatusSrv;
        changeRobotStatusSrv.request.toEnable = false;

        unsigned int commuRetryingCount_ = 0;

        while ((commuRetryingCount_ < communicationRetryingTimes_) && !(changeRobotStatusClientPtr->call(changeRobotStatusSrv))) {
            ++commuRetryingCount_;
            showMsg("Retrying to call changeRobotStatus service...");
            sleep(1);
        }
        if (commuRetryingCount_ >= communicationRetryingTimes_)
            ROS_ERROR("Fail to call changeRobotStatus service!");
    }

    /* 更改坐标变换时的时间积分量大小，不论是否成功执行（不成功是因为服务没响应），第二个if里的语句只执行一次。
     */
    void checkAndChangeTimeIntegration()
    {
        if (timeIntegrationChanged_)
            return;
        if (ibvsPtr->isDesiredPosNear())
        {
            aaw_ros::ChangeTimeIntegration changeTimeIntegSrv;
            changeTimeIntegSrv.request.TimeIntegration = lowVelTimeIntegration_;

            unsigned int commuRetryingCount_ = 0;

            while ((commuRetryingCount_ < communicationRetryingTimes_) && !(changeTimeIntegrationClientPtr->call(changeTimeIntegSrv))) {
                ++commuRetryingCount_;
                showMsg("Retrying to call \"change_time_integration_service\" ...");
                sleep(1);
            }
            if (commuRetryingCount_ >= communicationRetryingTimes_)
                ROS_WARN("Fail to call \"change_time_integration_service\".");

            timeIntegrationChanged_ = true;
        }
    }

    /**
     * 控制插入插销的动作，已经完成所有可能的错误尝试，返回０就直接退出程序，返回１代表已经完成动作。
     */
    int insertBolt()
    {
        aaw_ros::BoltMotionCtrl boltMotionCtrlSrv;
        boltMotionCtrlSrv.request.toInsert = true;

        unsigned int commuRetryingCount_ = 0;

        while ((commuRetryingCount_ < communicationRetryingTimes_) && !(boltMotionCtrlClientPtr->call(boltMotionCtrlSrv))) {
            ++commuRetryingCount_;
            showMsg("Retrying to call boltMotionCtrl service...");
            sleep(1);
        }

        if (commuRetryingCount_ >= communicationRetryingTimes_) {
            ROS_ERROR("Fail to call boltMotionCtrl service!");
            return 0;
        }
        else {
            ROS_INFO("Feedback from server: %d", boltMotionCtrlSrv.response.ExecStatus);
            return boltMotionCtrlSrv.response.ExecStatus;
        }
    }

    /**
     * 控制拔出插销的动作，已经完成所有可能的错误尝试，返回０就直接退出程序，返回１代表已经完成动作。
     */
    int pullOutBolt()
    {
        aaw_ros::BoltMotionCtrl boltMotionCtrlSrv;
        boltMotionCtrlSrv.request.toInsert = false;

        unsigned int commuRetryingCount_ = 0;

        while ((commuRetryingCount_ < communicationRetryingTimes_) && !(boltMotionCtrlClientPtr->call(boltMotionCtrlSrv))) {
            ++commuRetryingCount_;
            showMsg("Retrying to call boltMotionCtrl service...");
            sleep(1);
        }

        if (commuRetryingCount_ >= communicationRetryingTimes_) {
            ROS_ERROR("Fail to call boltMotionCtrl service!");
            return 0;
        }
        else {
            ROS_INFO("Feedback from server: %d", boltMotionCtrlSrv.response.ExecStatus);
            return boltMotionCtrlSrv.response.ExecStatus;
        }
    }

    /**
     * 控制推出LDS的动作，已经完成所有可能的错误尝试，返回０就直接退出程序，返回１代表已经完成动作。
     */
    int pushOutLDS()
    {
        aaw_ros::LDSMotionCtrl LDSMotionCtrlSrv;
        LDSMotionCtrlSrv.request.toPushOut = true;

        unsigned int commuRetryingCount_ = 0;

        while ((commuRetryingCount_ < communicationRetryingTimes_) && !(LDSMotionCtrlClientPtr->call(LDSMotionCtrlSrv))) {
            ++commuRetryingCount_;
            showMsg("Retrying to call LDSMotionCtrl service...");
            sleep(1);
        }

        if (commuRetryingCount_ >= communicationRetryingTimes_) {
            ROS_ERROR("Fail to call LDSMotionCtrl service!");
            return 0;
        }
        else {
            ROS_INFO("Feedback from server: %d", LDSMotionCtrlSrv.response.ExecStatus);
            return LDSMotionCtrlSrv.response.ExecStatus;
        }
    }

    /**
     * 控制收回LDS的动作，已经完成所有可能的错误尝试，返回０就直接退出程序，返回１代表已经完成动作。
     */
    int pullBackLDS()
    {
        aaw_ros::LDSMotionCtrl LDSMotionCtrlSrv;
        LDSMotionCtrlSrv.request.toPushOut = false;

        unsigned int commuRetryingCount_ = 0;

        while ((commuRetryingCount_ < communicationRetryingTimes_) && !(LDSMotionCtrlClientPtr->call(LDSMotionCtrlSrv))) {
            ++commuRetryingCount_;
            showMsg("Retrying to call LDSMotionCtrl service...");
            sleep(1);
        }

        if (commuRetryingCount_ >= communicationRetryingTimes_) {
            ROS_ERROR("Fail to call LDSMotionCtrl service!");
            return 0;
        }
        else {
            ROS_INFO("Feedback from server: %d", LDSMotionCtrlSrv.response.ExecStatus);
            return LDSMotionCtrlSrv.response.ExecStatus;
        }
    }

    void showMsg(const char * msg)
    {
        std::cerr<<msg<<"\n";
    }

    void errorMsg(const char * msg)
    {
        std::cerr<<msg<<"\n";
        exit(1);
    }
}

/* 此node接收视觉数据进行视觉伺服，并将控制量发送给moveRobotServer进行运动控制。
 */
int main(int argc, char** argv) {
    using namespace visualServo;
    ros::init(argc, argv, "aaw_visualServo");
    ros::NodeHandle nh_;

    message_filters::Subscriber<sensor_msgs::Image> left_image_sub_(nh_, "/image_raw/left", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_image_sub_(nh_, "/image_raw/right", 1);
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(left_image_sub_, right_image_sub_, 10);
    sync.registerCallback(boost::bind(&imageCb, _1, _2));

    ros::ServiceClient moveClient = nh_.serviceClient<aaw_ros::MoveRobot>("move_robot_input_camVel");
    ros::ServiceClient moveClient_DistanceZ = nh_.serviceClient<aaw_ros::MoveRobot_DistanceZ>("move_robot_input_distanceZ");
    ros::ServiceClient moveClient_CtrlVal = nh_.serviceClient<aaw_ros::MoveRobot_CtrlVal>("move_robot_input_ctrlVal");
    ros::ServiceClient changeRobotStatusClient = nh_.serviceClient<aaw_ros::ChangeRobotStatus>("change_robot_status_service");
    ros::ServiceClient changeTimeIntegrationClient = nh_.serviceClient<aaw_ros::ChangeTimeIntegration>("change_time_integration_service");
    ros::ServiceClient boltMotionCtrlClient = nh_.serviceClient<aaw_ros::BoltMotionCtrl>("bolt_motion_ctrl_service");
    ros::ServiceClient LDSMotionCtrlClient = nh_.serviceClient<aaw_ros::LDSMotionCtrl>("LDS_motion_ctrl_service");
    ros::ServiceClient updateCoordTransClient = nh_.serviceClient<aaw_ros::UpdateCoordTransformer>("update_coord_trans_service");

    ros::ServiceClient moveCarClient = nh_.serviceClient<interaction::MoveCar>("move_car_service");
    ros::ServiceClient adjustCarPosClient = nh_.serviceClient<interaction::AdjustCarPos>("adjust_car_pos_service");
    restartRobotMotion_ = nh_.advertiseService("restart_robot_motion_service", &restartRobotMotionCallback);

    weightSensorSub_ = nh_.subscribe("weigt_sensor_data", 1, &weightSensorDataCallback);
    robotCtrlValSub_ = nh_.subscribe("current_robot_ctrl_val", 1, &robotCtrlValCallback);

    moveClientPtr = &moveClient;
    moveClientPtr_DistanceZ = &moveClient_DistanceZ;
    moveClientPtr_CtrlVal = &moveClient_CtrlVal;
    changeRobotStatusClientPtr = &changeRobotStatusClient;
    changeTimeIntegrationClientPtr = &changeTimeIntegrationClient;
    boltMotionCtrlClientPtr = &boltMotionCtrlClient;
    LDSMotionCtrlClientPtr = &LDSMotionCtrlClient;
    updateCoordTransClientPtr = &updateCoordTransClient;
    moveCarClientPtr = &moveCarClient;
    adjustCarPosClientPtr = &adjustCarPosClient;

    ibvsPtr = new AAWIBVS(AAWIBVS::SN11818179);

    cv::namedWindow(Left_View, cv::WINDOW_NORMAL);
    cv::resizeWindow(Left_View, 800, 450);
    cv::namedWindow(Right_View, cv::WINDOW_NORMAL);
    cv::resizeWindow(Right_View, 800, 450);
    ros::Duration timer(0.1);

    while (ros::ok()) {
        ros::spinOnce();
        timer.sleep();
    }

    delete ibvsPtr;
    return 0;
}
