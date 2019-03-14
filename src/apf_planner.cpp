/*
 * Apf_planner node: Subscribes to /camera/obstacles2D defined in
 * obstacles_mapper_2d and compute the repulsive fields around an obstacle
 * (pixel = 1.0f in the obstacle matrix);
 *
 * Subscribes to /goalgenerator defined in goal_generator node
 * to set the goal to reach and compute the attractive field around the goal;
 *
 * Publishes data on topic /cmd_vel, sends infos about velocity, Odometry
 * moves the robot
 */


#include <../include/apf_planner.h>

    /*########################################################################
     * Inizializzazione lista parametri del costruttore
     * Nota: dichiarazione parametri esattamente nell'ordine in cui sono stati
     *       definiti nella classe; Altrimenti genera errore a run-time;
     *
     * Caveat: Inizializzazione lista atttributi della classe, piuttosto che
     *         dentro il corpo del costruttore, così da evitare ulteriore
     *         inizializzazione di default ridondante;
     *########################################################################
    */

apf_motion_planner::apf_motion_planner(ros::NodeHandle& nh) :

    nh_(nh)
{

}

/***************************************
Artificial Potential Fields function
****************************************/

geometry_msgs::Twist apf(const double& eta1, const double& eta2,const double& d_star, const double& Q_star)
{
    
}




void apf_motion_planner::init()
{
    pub_velocity_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    sub_odom_ = nh_.subscribe;

    sub_obstacle_mapper_ = nh_.subscribe<std_msgs::Float64MultiArray>("/camera/obstacles_mapper_2d");
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "apf_planner");

    ros::NodeHandle nh;

    apf_motion_planner apf_planner(nh);

    apf_planner.init();

    ros::spin();

    return 0;

}
