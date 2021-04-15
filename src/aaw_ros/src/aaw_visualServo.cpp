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
            if (withdrawAndGoHome()) {
                if (toDock_)
                    showMsg("Docking completed!");
                else
                    showMsg("Undocking completed!");
                taskFinished_ = true;
                disableRobot();
                waitAndWakeUpAction();
                return;
            }
            else {
                disableRobot();
                errorMsg("Task failed!");
            }
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

        ibvsPtr->updateVertexesCoordinates(vg4Left.get4Vertexes(), vg4Right.get4Vertexes());
        ibvsPtr->updateControlLaw();


        if (ibvsPtr->isDesiredPosArrived()) {
            // cv::destroyWindow(Left_View);
            // cv::destroyWindow(Right_View);

            //dock
            ROS_WARN("Desired pos arrived, now trying to dock, watch out!");
            sleep(timeWaitBeforeDocking_);

            unsigned int commuRetryingCount_ = 0;

            while((commuRetryingCount_ < communicationRetryingTimes_) && !dock()) {
                ++commuRetryingCount_;
                showMsg("Retrying to call moveRobotServer...");
                sleep(1);
            }
            if (commuRetryingCount_ >= communicationRetryingTimes_) {
                errorMsg("Communication with service \"move_robot_input_distanceZ\" failed!");
            }
            else {
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
        }
        else {
            checkAndChangeTimeIntegration();

            //visual servo 一旦开始伺服，到伺服结束之前，不允许进行其他形式的伺服控制请求（如ctrlVal）
            Eigen::Matrix<float, 6, 1> cameraVel;
            cameraVel = ibvsPtr->getCamCtrlVel();
            std::cout<<"Control velocity:\n"<<cameraVel<<std::endl;

            cv::imshow(Left_View, grayImageLeft);
            cv::imshow(Right_View, grayImageRight);

            unsigned int commuRetryingCount_ = 0;

            while ((commuRetryingCount_ < communicationRetryingTimes_) && !visualServoRobot(cameraVel)) {
                ++commuRetryingCount_;
                showMsg("Retrying visual servo...");
                sleep(1);
            }
            if (commuRetryingCount_ > communicationRetryingTimes_) {
                errorMsg("Visual servo error!");
            }
        }
        
        cv::waitKey(3);
    }

    bool wakeUpActionCallback()
    {
        toDock_ = !toDock_; //反转上方插销动作,其他的过程都一样，更新一下初始值即可。
        readyToGoHome_ = false;
        taskFinished_ = false;
        timeIntegrationChanged_ = false;
        enableRobot();
        updateCoordTrans();
    }

    /**
     * 请求更新坐标变换类对象，原先的已经不能用了，每一次新的伺服进行之前都需要更新，更新之前确保并联机器人已经在零点了。
     */
    int updateCoordTrans()
    {
        aaw_ros::UpdateCoordTransformer updateCoordTransSrv;
        updateCoordTransSrv.request.newTransformer = true;
        
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
        sleep(10);
        wakeUpActionCallback();
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

    /* 视觉伺服速度控制请求函数，执行失败时允许重复请求。
     */
    int visualServoRobot(Eigen::Matrix<float, 6, 1> camCtrlVel)
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

    /* 实现并联机构回撤、回零点控制。当上方的插销动作完成之后就调用该函数进行处理。
     */
    int withdrawAndGoHome()
    {
        // for (unsigned int i = keepDockingSecs_; i > 0; --i) {
        //     std::cout<<"Undocking countdown: "<<i<<"s\n";
        //     sleep(1);
        // }

        //事实上这里处理的不仅仅是通信失败的情况，还有并联机构收到指令但执行失败的情况
        //参见dock()和move2OriginalPos()函数，当通信成功时返回的是机器人返回指令的解析结果。
        //由于undock即使有重复发送的增量数据，过程也是安全的，因此允许多次尝试。
        unsigned int commuRetryingCount_ = 0;

        //trying to withdraw
        ROS_WARN("Now trying to withdraw, watch out!");
        while ((commuRetryingCount_ < communicationRetryingTimes_) && !withdraw()) {
            ++commuRetryingCount_;
            showMsg("Retrying to withdraw...");
            sleep(1);
        }
        if (commuRetryingCount_ >= communicationRetryingTimes_) {
            errorMsg("Withdraw failed!");
        }
        else {
            showMsg("Withdraw completed!");
        }

        //trying to go home
        commuRetryingCount_ = 0;
        showMsg("Now trying to go home...");
        while ((commuRetryingCount_ < communicationRetryingTimes_) && !move2OriginalPos()) {
            ++commuRetryingCount_;
            showMsg("Retrying to go home...");
            sleep(1);
        }
        if (commuRetryingCount_ >= communicationRetryingTimes_) {
            errorMsg("Going home failed!");
        }
        else {
            showMsg("Moved to original pos!");
            return 1;
        }
    }

    /*　并联机构向下收回动作。
     */
    int withdraw()
    {
        aaw_ros::MoveRobot_DistanceZ moveDistanceSrv;
        moveDistanceSrv.request.isUp = false;
        moveDistanceSrv.request.goDownDistance = moveDownDistance_;
        if (moveClientPtr_DistanceZ->call(moveDistanceSrv)) {
            ROS_INFO("Feedback from server: %d", moveDistanceSrv.response.ExecStatus);
            return moveDistanceSrv.response.ExecStatus;
        }
        else {
            ROS_ERROR("Failed to call service \"move_robot_input_distanceZ\"");
            return 0;
        }
    }

    /* 回零点控制。
     */
    int move2OriginalPos()
    {
        aaw_ros::MoveRobot_CtrlVal moveCtrlValSrv;
        moveCtrlValSrv.request.x = originalCtrlVal_[0];
        moveCtrlValSrv.request.y = originalCtrlVal_[1];
        moveCtrlValSrv.request.z = originalCtrlVal_[2];
        moveCtrlValSrv.request.a = originalCtrlVal_[3];
        moveCtrlValSrv.request.b = originalCtrlVal_[4];
        moveCtrlValSrv.request.c = originalCtrlVal_[5];

        if (moveClientPtr_CtrlVal->call(moveCtrlValSrv)) {
            ROS_INFO("Feedback from server: %d", moveCtrlValSrv.response.ExecStatus);
            return moveCtrlValSrv.response.ExecStatus;
        }
        else {
            ROS_ERROR("Failed to call service \"move_robot_input_ctrlVal\"");
            return 0;
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

    moveClientPtr = &moveClient;
    moveClientPtr_DistanceZ = &moveClient_DistanceZ;
    moveClientPtr_CtrlVal = &moveClient_CtrlVal;
    changeRobotStatusClientPtr = &changeRobotStatusClient;
    changeTimeIntegrationClientPtr = &changeTimeIntegrationClient;
    boltMotionCtrlClientPtr = &boltMotionCtrlClient;
    LDSMotionCtrlClientPtr = &LDSMotionCtrlClient;
    updateCoordTransClientPtr = &updateCoordTransClient;

    ibvsPtr = new AAWIBVS(AAWIBVS::SN11818179);

    cv::namedWindow(Left_View, cv::WINDOW_NORMAL);
    cv::resizeWindow(Left_View, 800, 450);
    cv::namedWindow(Right_View, cv::WINDOW_NORMAL);
    cv::resizeWindow(Right_View, 800, 450);
    ros::Duration timer(0.1);

    //不与小车协同才需要这一部分.
    showMsg("Visual servo will start in 5 senconds.");
    sleep(5);
    wakeUpActionCallback();

    while (ros::ok()) {
        ros::spinOnce();
        timer.sleep();
    }

    delete ibvsPtr;
    return 0;
}
