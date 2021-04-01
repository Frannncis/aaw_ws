#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "aawvertexesgainer.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <Eigen/Dense>
#include "aawibvs.h"
#include <aaw_opencv/MoveRobot.h>
#include <boost/bind.hpp>

static const std::string Left_View = "Left View";
static const std::string Right_View = "Right View";

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

ros::ServiceClient *moveClientPtr;

//image comes in as a ROS message, but gets converted to an OpenCV type
void imageCb(const sensor_msgs::ImageConstPtr& leftImage, const sensor_msgs::ImageConstPtr& rightImage) {
    cv_bridge::CvImagePtr cv_ptr_left, cv_ptr_right; //OpenCV data type
    try {
        cv_ptr_left = cv_bridge::toCvCopy(leftImage, sensor_msgs::image_encodings::BGR8);
        cv_ptr_right = cv_bridge::toCvCopy(rightImage, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    AawVertexesGainer vg4Left, vg4Right;
    Eigen::VectorXd cameraVel;
    AawIBVS ibvs(AawIBVS::SN11818179);
    cv::Mat grayImageLeft, grayImageRight;
    cv::cvtColor(cv_ptr_left->image, grayImageLeft, cv::COLOR_BGR2GRAY);
    cv::cvtColor(cv_ptr_right->image, grayImageRight, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(grayImageLeft, grayImageLeft, cv::Size(3,3),0,0);
    cv::GaussianBlur(grayImageRight, grayImageRight, cv::Size(3,3),0,0);
    vg4Left = AawVertexesGainer(grayImageLeft);
    vg4Right = AawVertexesGainer(grayImageRight);
    ibvs.updateVertexesCoordinates(vg4Left.get4Vertexes(), vg4Right.get4Vertexes());
    ibvs.updateControlLaw();
    cameraVel = ibvs.getControlVel();
    std::cout<<"Control velocity:\n"<<cameraVel<<std::endl;
    cv::imshow(Left_View, grayImageLeft);
    cv::imshow(Right_View, grayImageRight);

    aaw_opencv::MoveRobot moveSrv;
    moveSrv.request.vx = cameraVel(0);
    moveSrv.request.vy = cameraVel(1);
    moveSrv.request.vz = cameraVel(2);
    moveSrv.request.wx = cameraVel(3);
    moveSrv.request.wy = cameraVel(4);
    moveSrv.request.wz = cameraVel(5);
    ros::ServiceClient client = (ros::ServiceClient)*moveClientPtr;
    if (client.call(moveSrv)) {
        ROS_INFO("Feedback from server: %d", moveSrv.response.ExecStatus);
    }
    else {
        ROS_ERROR("Failed to call service \"move_robot_to_pos\"\n");
    }
    
    cv::waitKey(3);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "vertexes_tracker");
    ros::NodeHandle nh_;

    message_filters::Subscriber<sensor_msgs::Image> left_image_sub_(nh_, "/image_raw/left", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_image_sub_(nh_, "/image_raw/right", 1);
    TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(left_image_sub_, right_image_sub_, 10);
    sync.registerCallback(boost::bind(&imageCb, _1, _2));

    ros::ServiceClient moveClient = nh_.serviceClient<aaw_opencv::MoveRobot>("move_robot_to_pos");
    moveClientPtr = &moveClient;

    cv::namedWindow(Left_View, cv::WINDOW_NORMAL);
    cv::resizeWindow(Left_View, 800, 450);
    cv::namedWindow(Right_View, cv::WINDOW_NORMAL);
    cv::resizeWindow(Right_View, 800, 450);
    ros::Duration timer(0.1);

    while (ros::ok()) {
        ros::spinOnce();
        timer.sleep();
    }
    return 0;
}
