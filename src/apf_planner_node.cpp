#include <../include/apf_planner.h>

int main(int argc, char** argv)
{

    ros::init(argc, argv, "apf_planner");

    ros::NodeHandle nh;

    apf_motion_planner apf_planner(nh);

    apf_planner.init();

    ros::spin();

    return 0;

}
