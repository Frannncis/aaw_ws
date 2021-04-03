#ifndef AAW_VISUAL_SERVO_H_
#define AAW_VISUAL_SERVO_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include "aawvertexesgainer.h"
#include "aawibvs.h"
#include <aaw_ros/MoveRobot.h>
#include <aaw_ros/MoveRobot_DistanceZ.h>
#include <aaw_ros/MoveRobot_CtrlVal.h>
#include <aaw_ros/DisableRobot.h>
#include <boost/bind.hpp>
#include "aaw_originalCtrlVal.h"

static const std::string Left_View = "Left View";
static const std::string Right_View = "Right View";
const unsigned int timeWaitBeforeDocking_ = 3;   //seconds

bool isDockingCompleted_ = false;
bool criticalError_ = false;
bool taskFinished_ = false;
const float moveUpDistance_ = 86.09861;
const float moveDownDistance_ = moveUpDistance_;
const unsigned int keepDockingSecs_ = 30;
const unsigned int communicationRetryingTimes_ = 5;

ros::ServiceClient *moveClientPtr;
ros::ServiceClient *moveClientPtr_DistanceZ;
ros::ServiceClient *moveClientPtr_CtrlVal;
ros::ServiceClient *disableRobotClientPtr;
AAWIBVS *ibvsPtr;

void imageCb(const sensor_msgs::ImageConstPtr& leftImage, const sensor_msgs::ImageConstPtr& rightImage);

int visualServoRobot(Eigen::Matrix<float, 6, 1> camCtrlVel);
int dock();

int waitingAndWithdraw();
int undock();
int move2OriginalPos();
void disableRobot();

#endif