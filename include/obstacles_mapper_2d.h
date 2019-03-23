#pragma once

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <pcl/console/parse.h>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/eigen.hpp>
//#include <DepthImageToLaserScan.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/passthrough.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/common/common_headers.h>
#include <std_msgs/Float64MultiArray.h>
#include <pcl/range_image/range_image.h>
#include <eigen_conversions/eigen_msg.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <pointcloud_to_laserscan_nodelet.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>


#define PI 3.14159265

//using namespace pcl;
//using namespace cv;

class obstacles_mapper_2d
{

public:

    //constructor (save node handle)
    obstacles_mapper_2d(ros::NodeHandle& nh);

    //create publisher, subscribes to desired topic (start routine)
    void init();

    //handle callback for the subscribed topic
    void pclCallback(const sensor_msgs::PointCloud2::ConstPtr& pc);

    // resolution parameters
    int cell_dimension_mm;
    int desired_depth_mm;
    int desired_width_mm;

protected:

    ros::NodeHandle nh_;

    ros::Subscriber sub_;

    ros::Publisher  pub_;

};
