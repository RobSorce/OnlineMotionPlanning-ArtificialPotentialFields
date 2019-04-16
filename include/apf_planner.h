#pragma once

#include <cmath>
#include <vector>
#include <limits>
#include <sstream>
#include <iostream>
#include <ros/ros.h>
#include <Eigen/Core>
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/eigen.hpp>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <std_msgs/Float64MultiArray.h>
#include <eigen_conversions/eigen_msg.h>

// ROS/tf libraries
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

typedef std::vector<cv::Point> ObstacleInfo;

enum RepulsiveType {REPULSIVE, VORTEX};

class apf_motion_planner
{

public:

    //Constructor
    //r_type: DEFAULT = REPULSIVE;
    apf_motion_planner(ros::NodeHandle& nh, RepulsiveType r_type = RepulsiveType::REPULSIVE);

    void apfCallback(const std_msgs::Float64MultiArray::ConstPtr& obs);

    void generate_potential_map(const cv::Mat& obstacles_map, const std::vector<ObstacleInfo>& obstacles);

    //initialize publisher/subscriber
    void init();

    //compute potential fields
    geometry_msgs::Twist attractive_potential(float xgoal, float ygoal, float xrobot, float yrobot);

    geometry_msgs::Twist repulsive_potential(float xr, float yr, float xo, float yo);

    geometry_msgs::Twist vortex_potential(float xr, float yr, float xo, float yo);

    geometry_msgs::Twist artificial_potential_fields(const std::vector<ObstacleInfo>& obstacles_array, float xr, float yr, float xg, float yg);

    /***************************************************************************
    * Variables for Artificial Potential Fields formula                        *
    *                                                                          *
    ***************************************************************************/

    double k_attractive;
    double k_repulsive;
    double k_theta;

    double gamma; // >1;
    double eta_0;
    double rho;

protected:

    RepulsiveType r_type;

    ros::NodeHandle nh_;

    ros::Publisher  pub_velocity_;        //Publishes on (/cmd_vel)

    ros::Subscriber sub_obstacle_mapper_; //Subscribes to (/obstacles_mapper_2d)

    geometry_msgs::Twist vel_;

};
