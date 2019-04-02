#pragma once

#include <cmath>					// to use: atan2, sqrt, pow, etc
#include <vector>
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
#include <tf/transform_listener.h>		// to use: tf listner
#include <tf/transform_datatypes.h>		// to use: tf datatypes
#include <tf/transform_broadcaster.h>	// to use: tf broadcaster


typedef std::vector<cv::Point> ObstacleInfo;

enum RepulsiveType {REPULSIVE, VORTEX};

class apf_motion_planner
{

public:

    //Constructor
    apf_motion_planner(ros::NodeHandle& nh, RepulsiveType r_type = RepulsiveType::REPULSIVE);

    void apfCallback(const std_msgs::Float64MultiArray::ConstPtr& obs);

    void generate_potential_map(const cv::Matif& obstacles_map);

    //initialize publisher/subscriber
    void init();

    //compute potential fields
    geometry_msgs::Twist attractive_potential(float xgoal, float ygoal, float xrobot, float yrobot);

    geometry_msgs::Twist repulsive_potential(float xr, float yr, float xo, float yo);

    geometry_msgs::Twist vortex_potential(float xr, float yr, float xo, float yo);

    geometry_msgs::Twist artificial_potential_fields(const std::vector<ObstacleInfo>& obstacles_array, float xr, float yr);


    /***************************************************************************
    * Variables for Artificial Potential Fields formula
    *
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

    //ros::Subscriber sub_odom_;            //Subscribes to (/odom)

    //tf::TransformListener listener_;

    // robot's origin w.r.t. "base_link"
	//tf::Stamped<tf::Pose> robot_pose;

	// robot's origin w.r.t. "odom"
	//tf::Stamped<tf::Pose> tf_robot_odom_pose;

    geometry_msgs::Twist vel_;

};

// template <typename RepulsiveType>
// geometry_msgs::Twist apf_motion_planner::artificial_potential_fields(const std::vector<ObstacleInfo>& obstacles_array, float xr, float yr, RepulsiveType repulsive_field_function)
// {
//     /***************************************************************************
//     * Local variables for Artificial Potential Fields formula
//     *
//     ***************************************************************************/
//
//     geometry_msgs::Twist attractive_vel;
//     geometry_msgs::Twist repulsive_vel;
//     geometry_msgs::Twist total_vel;
//
// 	double repulsive_potential_x;
//     double repulsive_potential_y;
//     double repulsive_potential_theta;
//
//     std::vector<cv::Point> obstacle_closest_points(obstacles_array.size());
//
//     Eigen::Vector2f goal(xr, yr + 1000);
//
//     /*******************************************************************
//      * chiamata a funzione attractive potential: salvo i dati          *
//      * geometry_msgs::Twist nella variabile attractive_vel;            *
//      *******************************************************************/
//     attractive_vel = attractive_potential(goal.x(), goal.y(), xr, yr);
//
//
//     int o_idx = 0;
//     cv::Point robot_position(xr, yr);
//
//     for(const ObstacleInfo& obstacle : obstacles_array) {
//         double old_distance = std::numeric_limits<double>::max(); //std::static_cast<double>((1 << 31) - 1); //  ~MAX_INT
//
//         for(const cv::Point& obstacle_point : obstacle ) {
//
//             double new_distance = cv::norm( cv::Mat(obstacle_point), cv::Mat(robot_position));
//
//             if(new_distance < old_distance) {
//                 obstacle_closest_points[o_idx] = obstacle_point;
//                 old_distance = new_distance;
//             }
//         }
//         ++o_idx;
//     }
//
//      /***********************************************
//      * Repulsive Potential                          *
//      ************************************************/
//      for(const cv::Point& obstacle : obstacle_closest_points) {
//          repulsive_vel += repulsive_field_function(goal.x(), goal.y(), obstacle.x(), obstacle.y());
//      }
//
//      //Sommatoria di tutte le forze attrattive + repulsive agenti sulle coordinate
//      total_vel.linear.x  = repulsive_vel.linear.x  + attractive_vel.linear.x;
//      total_vel.linear.y  = repulsive_vel.linear.y  + attractive_vel.linear.y;
//      total_vel.angular.z = repulsive_vel.angular.z + attractive_vel.angular.z;
//
//      ///////////////////////////////////////////////////////////////////////////
//      //Print vel data
//      //std::cerr << vel << '\n';
//      ///////////////////////////////////////////////////////////////////////////
//
//      return total_vel;
// }
