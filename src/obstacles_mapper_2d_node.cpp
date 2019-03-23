#pragma once

#include <../include/obstacles_mapper_2d.h>

int main(int argc, char** argv)
{

    ros::init(argc, argv, "obstacles_mapper_2d");

    ros::NodeHandle nh;

    obstacles_mapper_2d obstacles_mapper_2d(nh);

    /*
     * Modify resolution parameters
     * Unit = milimeters;
     */

    obstacles_mapper_2d.cell_dimension_mm = 10;
    //obstacles_mapper_2d.desired_depth_mm = 6000;
    //obstacles_mapper_2d.desired_width_mm = 10000;
    obstacles_mapper_2d.init();

    ros::spin();

    return 0;

}
