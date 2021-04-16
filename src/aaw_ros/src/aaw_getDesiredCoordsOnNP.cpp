#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <Eigen/Dense>
// #include "aawvertexesgainer.h"
#include "aawibvs.h"
#include <aaw_ros/MoveRobot.h>
#include <boost/bind.hpp>

#include "aawvertexesgainer2.h"

static const std::string Left_View = "Left View";
static const std::string Right_View = "Right View";

AAWIBVS *ibvsPtr;

void imageCb(const sensor_msgs::ImageConstPtr& leftImage, const sensor_msgs::ImageConstPtr& rightImage) {
    cv_bridge::CvImagePtr cv_ptr_left, cv_ptr_right;
    try {
        cv_ptr_left = cv_bridge::toCvCopy(leftImage, sensor_msgs::image_encodings::BGR8);
        cv_ptr_right = cv_bridge::toCvCopy(rightImage, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    /*
    AAWVertexesGainer vg4Left, vg4Right;
    cv::Mat grayImageLeft, grayImageRight;
    cv::cvtColor(cv_ptr_left->image, grayImageLeft, cv::COLOR_BGR2GRAY);
    cv::cvtColor(cv_ptr_right->image, grayImageRight, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(grayImageLeft, grayImageLeft, cv::Size(3,3),0,0);
    cv::GaussianBlur(grayImageRight, grayImageRight, cv::Size(3,3),0,0);
    vg4Left = AAWVertexesGainer(grayImageLeft);
    vg4Right = AAWVertexesGainer(grayImageRight);
    */

    AAWVertexesGainer2 vg4Left, vg4Right;
    cv::Mat grayImageLeft, grayImageRight;
    cv::cvtColor(cv_ptr_left->image, grayImageLeft, cv::COLOR_BGR2GRAY);
    cv::cvtColor(cv_ptr_right->image, grayImageRight, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(grayImageLeft, grayImageLeft, cv::Size(3,3),0,0);
    cv::GaussianBlur(grayImageRight, grayImageRight, cv::Size(3,3),0,0);
    vg4Left = AAWVertexesGainer2(grayImageLeft);
    vg4Right = AAWVertexesGainer2(grayImageRight);
    // if (vg4Left.isLeftBoundOutaView())
    // {
    //     ROS_WARN("LeftView: left bound out of view!");
    // }
    // if (vg4Left.isRightBoundOutaView()) {
    //     ROS_WARN("LeftView: right bound out of view!");
    // }

    // if (vg4Right.isLeftBoundOutaView())
    // {
    //     ROS_WARN("RightView: left bound out of view!");
    // }
    // if (vg4Right.isRightBoundOutaView()) {
    //     ROS_WARN("RightView: right bound out of view!");
    // }
    

    ibvsPtr->updateVertexesCoordinates(vg4Left.get4Vertexes(), vg4Right.get4Vertexes());
    ibvsPtr->measureDesiredCoordsOnNP();    //起主要作用的函数调用
    cv::imshow(Left_View, grayImageLeft);
    cv::imshow(Right_View, grayImageRight);

    cv::waitKey(3);
}

/* 本node用来获取期望对准位置处相机视野中特征点的数据，调整到期望位姿后启动本node的相应的launch文件，待数据输出后更新到aawibvs.h文件中即可。
 * 
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "aaw_getDesiredCoordsOnNP");
    ros::NodeHandle nh_;

    message_filters::Subscriber<sensor_msgs::Image> left_image_sub_(nh_, "/image_raw/left", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_image_sub_(nh_, "/image_raw/right", 1);
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(left_image_sub_, right_image_sub_, 10);
    sync.registerCallback(boost::bind(&imageCb, _1, _2));

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
