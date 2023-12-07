#ifndef TOPIC_HPP
#define TOPIC_HPP

#include "time.h"  
#include "string.h"  
#include "sstream"
#include "termio.h"
#include <iostream>
#include <vector>
#include <csignal>
#include "ros/ros.h"  
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "box_ros_msgs/BoundingBoxes.h"
#include "kbd_ros_msgs/kbd.h"
#include "boost/thread.hpp"
#include "cv_bridge/cv_bridge.h"
#include  "opencv2/opencv.hpp"
#include  "opencv2/highgui.hpp"
#include  "tf/tf.h"
#include "cmath"
#include <algorithm> 
#include <yaml-cpp/yaml.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#define joint_NUM 6

using namespace ros;
using namespace std;
using namespace std_msgs;
using namespace geometry_msgs;
using namespace sensor_msgs;
using namespace cv;
using namespace  cv_bridge;

typedef struct
{
    int id;
    char name;
    cv::Point point;
    cv::Point3d Point3;
    cv::Point point_tl;
    cv::Point point_tr;
    cv::Point point_bl;
    cv::Point point_br;
}color_;

typedef struct
{
    int sum;
    int red;
    int blue;
    int yellow;
}count_;

typedef struct
{
    count_ count;
    int sign;//1 : L:blue R:red;       0 : L:red R:blue;
    color_ red[30];
    color_ blue[30];
    color_ yellow[30];
}box_;

typedef struct
{
    char name[joint_NUM][36];      
    double  position[joint_NUM];
    double  velocity[joint_NUM];
    double  effort[joint_NUM];
}joint_states_;

typedef struct 
{
    Twist pub_cmd_vel;
    Float64 pub_left_string;
    Float64 pub_right_string;
}car_ctrl_;

typedef struct 
{
    cv::Mat R=Mat::zeros(3,3,CV_64F);
    cv::Mat T=Mat::zeros(3,1,CV_64F);
    cv::Mat K=Mat::zeros(3,3,CV_64F);
    cv::Mat distCoeff = Mat::zeros(5,1,CV_64F);
}matrix_;

typedef struct 
{
    cv::Point2d point2d;
    double distance;
}Point2d_;

Mat polyfit(vector<cv::Point>& in_point, int n , char x);
car_ctrl_ car_ctrl(char direction,float angle);
void point_sort(cv::Point2d point[], int num);
int factorial(int n);
int point_distance(color_ color[] , char sign);
cv::Mat matrix_R(double x,double y, double z);
cv::Point3d Towordpoint(cv::Point P2, int h);
cv::Point Touvpoint(int x , int y , int z);
int test_point(Point2d_ A,Point2d_ B, double x_range=1000,double y_range=1000);
cv::Point convert(cv::Point2d p_uv,int width,int height,double x_convert,double y_convert);
Point2d_ ToPoint2d(color_ box , cv::Point2d car);
void roadblock_point_w(vector<Point2d_> *point_w , Point2d_ P , double distance_limt);
void roadblock_box(Mat *src,color_ color,const cv::Scalar &scalar);
void roadblock_point_c(vector<Point2d> *point_c , vector<Point2d_> point_w , double x , double y , double angle);
vector<cv::Point2d> car_target(vector<cv::Point2d> red_point_c , vector<cv::Point2d> blue_point_c);
#endif