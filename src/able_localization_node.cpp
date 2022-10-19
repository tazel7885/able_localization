#include <ros/ros.h>
#include <ros/package.h>

#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <utility>
#include <set>
#include <stdio.h>
#include <fstream>
 
const double math_pi = 3.141592;
const double wheel_base = 0.338;

geometry_msgs::Pose2D able_odom;
geometry_msgs::Pose2D aruco_position;

float velR = 0.0, velL = 0.0;
bool aruco_check = false;

void PositionCallback(const geometry_msgs::Pose2D::ConstPtr &msg)
{
    aruco_position = *msg;
}

void CheckCallback(const std_msgs::Bool::ConstPtr &msg)
{
    aruco_check = msg->data;
}

void VelRCallback(const std_msgs::Float32::ConstPtr &msg)
{
    velR = msg->data;
}

void VelLCallback(const std_msgs::Float32::ConstPtr &msg)
{
    velL = msg->data;
}

void CalcAblePosition()
{
    if(aruco_check)
    {
        able_odom = aruco_position;
    }
    else
    {
        float linear_vel = (velL + velR) / 2;
        float angular_vel = (velR - velL) / wheel_base;

        float distance_delta = linear_vel * 0.1;
        float angular_delta = angular_vel * 0.1;

        able_odom.x = able_odom.x + distance_delta * cos(able_odom.theta + angular_delta / 2);
        able_odom.y = able_odom.y + distance_delta * sin(able_odom.theta + angular_delta / 2);

        able_odom.theta += angular_delta;

        if(able_odom.theta < 0)
        {
            able_odom.theta = able_odom.theta + 2 * math_pi;
        }
        else if(able_odom.theta > 2 * math_pi)
        {
            able_odom.theta = able_odom.theta - 2 * math_pi;
        }

        if(able_odom.theta > math_pi)
        {
            able_odom.theta = able_odom.theta - 2 * math_pi;
        }
    }
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "able_localization_node");
    ros::NodeHandle nh;

    ros::Rate loop_rate(10);

    ros::Publisher pub_able_odom = nh.advertise<geometry_msgs::Pose2D>("/heroehs/able/odom", 10);

    ros::Subscriber sub_aruco_position = nh.subscribe("/heroehs/able/aruco_marker/position", 10, PositionCallback);
    ros::Subscriber sub_aruco_check = nh.subscribe("/heroehs/able/aruco_marker/check", 10, CheckCallback);
    ros::Subscriber sub_mobile_vel_R = nh.subscribe("/heroehs/able/mobile/velR", 10, VelRCallback);
    ros::Subscriber sub_mobile_vel_L = nh.subscribe("/heroehs/able/mobile/velL", 10, VelLCallback);

    able_odom.x = 0.0;
    able_odom.y = 0.0;
    able_odom.theta = 0.0;
    
    while (ros::ok())
    {
        CalcAblePosition();
        pub_able_odom.publish(able_odom);
        
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}