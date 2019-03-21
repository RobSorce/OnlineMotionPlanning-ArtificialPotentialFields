#include <cmath>					// to use: atan2, sqrt, pow, etc
#include <vector>
#include <iomanip>     				// to use: setprecision()
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



class apf_motion_planner
{

public:
    //Constructor
    apf_motion_planner(ros::NodeHandle& nh);

    //get MARRtino current pose
    void get_MARRtino_pose();

    //initialize publisher/subscriber
    void init();

    //compute potential fields
    geometry_msgs::Twist apf(const double& k_attractive, const double& k_repulsive,
                             const double& rho, const double& eta_0,
                             std_msgs::Float64MultiArray map_info);

    /***************************************************************************
    * Variables for Artificial Potential Fields formula
    *
    ***************************************************************************/

    const double k_attractive;
    const double k_repulsive;
    const double k_theta;

    const int gamma; // >1;
    double    eta_0;
    double    rho;

protected:

    ros::NodeHandle nh_;

    ros::Publisher  pub_velocity_;        //Publishes on (/cmd_vel)

    ros::Subscriber sub_obstacle_mapper_; //Subscribes to (/obstacles_mapper_2d)

    //ros::Subscriber sub_odom_;            //Subscribes to (/odom)

    //tf::TransformListener listener_;

    // robot's origin w.r.t. "base_link"
	//tf::Stamped<tf::Pose> robot_pose;

	// robot's origin w.r.t. "odom"
	//tf::Stamped<tf::Pose> tf_robot_odom_pose;

};
