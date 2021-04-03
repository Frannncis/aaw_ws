#include "aaw_visualServo.h"

void imageCb(const sensor_msgs::ImageConstPtr& leftImage, const sensor_msgs::ImageConstPtr& rightImage) {
    if (criticalError_ || taskFinished_)
        return;

    if (isDockingCompleted_) {
        if (waitingAndWithdraw()) {
            std::cout<<"Task finished!\n";
            taskFinished_ = true;
            disableRobot();
            return;
        }
        else {
            ROS_ERROR("Task failed!");
            criticalError_ = true;
            disableRobot();
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
        //visual servo
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
 * 目前只有出错就退出程序这种处理方法。
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

int waitingAndWithdraw()
{
    for (unsigned int i = keepDockingSecs_; i > 0; --i) {
        std::cout<<"Undocking countdown: "<<keepDockingSecs_<<"s\n";
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

int main(int argc, char** argv) {
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

    moveClientPtr = &moveClient;
    moveClientPtr_DistanceZ = &moveClient_DistanceZ;
    moveClientPtr_CtrlVal = &moveClient_CtrlVal;
    disableRobotClientPtr = &disableRobotClient;

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
