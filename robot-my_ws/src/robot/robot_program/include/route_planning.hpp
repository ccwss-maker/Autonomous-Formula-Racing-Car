#ifndef ROUTE_PLANNING_HPP
#define ROUTE_PLANNING_HPP

#include <iostream>
#include  "opencv2/opencv.hpp"
#include  "opencv2/highgui.hpp"
#include "cmath"

typedef struct{
    double angle_R;
    double angle_L;
    cv::Point2d p_fit;
}angle_;

typedef struct{
    double car_L;
    double car_W;
    double speed;
}CarState_;

angle_ followPurePursuit(CarState_ Carstate  , std::vector<cv::Point2d> p_target  , double kv , double d0);
cv::Point2d find_p_target(std::vector<cv::Point2d> p_target , cv::Point2d p_center , double ld);
double find_distance(cv::Point2d P1 , cv::Point2d P2);

#endif