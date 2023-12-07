#include <route_planning.hpp>
#include <iostream>

angle_ followPurePursuit(CarState_ Carstate  , std::vector<cv::Point2d> p_target  , double kv , double d0) 
{
    double ld = kv*Carstate.speed+Carstate.car_L*d0;
    cv::Point2d p_center(0 , -Carstate.car_L / 2);

    for(int i = 0 ; i < p_target.size() ; i++)
    {
        p_target[i].x/=1000;
        p_target[i].y/=1000;
    }
    cv::Point2d p_target_fix = find_p_target(p_target , p_center , ld);

    double ey = p_target_fix.x;
    ld = find_distance(p_target_fix , p_center);

    double angle = atan(2.0*Carstate.car_L*ey/pow(ld,2));

    double R = ld/2/sin(angle);

    angle_ angle_wheel;
    angle_wheel.angle_L = atan(Carstate.car_L / (R - Carstate.car_W) / 2);
    angle_wheel.angle_R = atan(Carstate.car_L / (R + Carstate.car_W) /2);
    angle_wheel.p_fit = p_target_fix;
    return angle_wheel;
}

cv::Point2d find_p_target(std::vector<cv::Point2d> p_target , cv::Point2d p_center , double ld)
{
    struct target
    {
        cv::Point2d p;
        int sign = 0;
        double distance;
    };

    struct target max;
    struct target min;

    max.distance = 0;
    min.distance = 100;
    
    cv::Point2d p_target_fit;

    for(int i=0 ; i < p_target.size() ; i++)
    {
        double distance = find_distance(p_target[i] , p_center);
        if(distance <= ld && distance >= max.distance)
        {
            max.distance = distance;
            max.p = p_target[i];
            max.sign = 1;
        }
        else if(distance > ld && distance <= min.distance)
        {
            min.distance = distance;
            min.p = p_target[i];
            min.sign = 1;
        }
    }
    if(max.sign == 1 && min.sign == 1)
    {
        double distance_max , distance_min;
        distance_min = find_distance(min.p , p_center) - ld;
        distance_max = ld - find_distance(max.p , p_center);
        p_target_fit = distance_max < distance_min ? max.p : min.p;
    }
    else if(max.sign == 1 && min.sign == 0)
    {
        p_target_fit = max.p;
    }
    else if(max.sign == 0 && min.sign == 1)
    {
        p_target_fit = min.p;
    }

    return p_target_fit;
}

double find_distance(cv::Point2d P1 , cv::Point2d P2)
{
    double d_x = P1.x - P2.x;
    double d_y = P1.y - P2.y;
    double distance = pow(pow(d_x,2)+pow(d_y,2) , 0.5);

    return distance;
}
