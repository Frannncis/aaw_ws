#include "aaw_visualServo.h"

namespace visualServo {
    /* 相机图像话题的回调函数，是本node的主循环结构。
    * 主要完成图像特征提取与视觉伺服。
    * 在视觉伺服完成之前进行伺服控制，完成之后进行对接、等待、分离、下使能机器人等操作。
    */
    void imageCb(const sensor_msgs::ImageConstPtr& leftImage, const sensor_msgs::ImageConstPtr& rightImage) {
        if (criticalError_ || taskFinished_)
            return;

        if (isDockingCompleted_) {
            if (waitingAndWithdraw()) {
                std::cout<<"Task finished!\n";
                taskFinished_ = true;
                disableRobot();
                exit(0);
                return;
            }
            else {
                ROS_ERROR("Task failed!");
                criticalError_ = true;
                disableRobot();
                exit(1);
                return;
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

        AAWVertexesGainer vg4Left, vg4Right;
        vg4Left = AAWVertexesGainer(grayImageLeft);
        vg4Right = AAWVertexesGainer(grayImageRight);

        ibvsPtr->updateVertexesCoordinates(vg4Left.get4Vertexes(), vg4Right.get4Vertexes());
        ibvsPtr->updateControlLaw();
        if (ibvsPtr->isDesiredPosArrived()) {
            cv::destroyWindow(Left_View);
            cv::destroyWindow(Right_View);

            //dock
            ROS_WARN("Desired pos arrived, now trying to dock, watch out!");
            sleep(timeWaitBeforeDocking_);

            unsigned int commuRetryingCount_ = 0;

            while((commuRetryingCount_ < communicationRetryingTimes_) && !dock()) {
                ++commuRetryingCount_;
                std::cout<<"Retrying to call moveRobotServer...\n";
                sleep(1);
            }
            if (commuRetryingCount_ >= communicationRetryingTimes_) {
                ROS_ERROR("Communication with service \"move_robot_input_distanceZ\" failed!");
                criticalError_ = true;
                return;
            }
            else {
                std::cout<<"Docking completed!\n";
                isDockingCompleted_ = true;
                return;
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
                std::cout<<"Retrying visual servo...\n";
                sleep(1);
            }
            if (commuRetryingCount_ > communicationRetryingTimes_) {
                ROS_ERROR("Visual servo error!");
                criticalError_ = true;
                return;
            }
        }
        
        cv::waitKey(3);
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
                criticalError_ = true;
                ROS_ERROR("Something terrible happend, docking failed, exiting...");
                exit(1);
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

    /* 实现等待、分离、回零点控制。当dock()完成之后就调用该函数进行处理。
    */
    int waitingAndWithdraw()
    {
        for (unsigned int i = keepDockingSecs_; i > 0; --i) {
            std::cout<<"Undocking countdown: "<<i<<"s\n";
            sleep(1);
        }

        //事实上这里处理的不仅仅是通信失败的情况，还有并联机构收到指令但执行失败的情况
        //参见dock()和move2OriginalPos()函数，当通信成功时返回的是机器人返回指令的解析结果。
        //由于undock即使有重复发送的增量数据，过程也是安全的，因此允许多次尝试。
        unsigned int commuRetryingCount_ = 0;

        //trying to undock
        ROS_WARN("Now trying to undock, watch out!");
        while ((commuRetryingCount_ < communicationRetryingTimes_) && !undock()) {
            ++commuRetryingCount_;
            std::cout<<"Retrying to undock...\n";
            sleep(1);
        }
        if (commuRetryingCount_ >= communicationRetryingTimes_) {
            ROS_ERROR("Undocking failed!");
            return 0;
        }
        else {
            std::cout<<"Undocking completed!\n";
            commuRetryingCount_ = 0;
        }

        //trying to move to the original pos
        std::cout<<"Now trying to move to the original pos...\n";
        while ((commuRetryingCount_ < communicationRetryingTimes_) && !move2OriginalPos()) {
            ++commuRetryingCount_;
            std::cout<<"Retrying to move to original pos...\n";
            sleep(1);
        }
        if (commuRetryingCount_ >= communicationRetryingTimes_) {
            ROS_ERROR("Moving to original pos failed!");
            return 0;
        }
        else {
            std::cout<<"Moved to original pos!\n";
            return 1;
        }
    }

    /* 分离控制。
    */
    int undock()
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

    /* 下使能机器人控制函数，在完成waitingAndWithdraw()之后，不论是否执行成功，都调用该函数下使能机器人。
    */
    void disableRobot()
    {
        aaw_ros::DisableRobot disableRobotSrv;
        disableRobotSrv.request.req = 1;

        unsigned int commuRetryingCount_ = 0;

        while ((commuRetryingCount_ < communicationRetryingTimes_) && !(disableRobotClientPtr->call(disableRobotSrv))) {
            ++commuRetryingCount_;
            std::cout<<"Retrying to call disableRobot service...\n";
            sleep(1);
        }
        if (commuRetryingCount_ >= communicationRetryingTimes_)
            ROS_WARN("Fail to call disableRobot service, please disable it manually.\n");
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
                std::cout<<"Retrying to call \"change_time_integration_service\" ...\n";
                sleep(1);
            }
            if (commuRetryingCount_ >= communicationRetryingTimes_)
                ROS_WARN("Fail to call \"change_time_integration_service\".\n");

            timeIntegrationChanged_ = true;
        }
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
    ros::ServiceClient disableRobotClient = nh_.serviceClient<aaw_ros::DisableRobot>("disable_robot_service");
    ros::ServiceClient changeTimeIntegrationClient = nh_.serviceClient<aaw_ros::ChangeTimeIntegration>("change_time_integration_service");

    moveClientPtr = &moveClient;
    moveClientPtr_DistanceZ = &moveClient_DistanceZ;
    moveClientPtr_CtrlVal = &moveClient_CtrlVal;
    disableRobotClientPtr = &disableRobotClient;
    changeTimeIntegrationClientPtr = &changeTimeIntegrationClient;

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
