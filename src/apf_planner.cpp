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

    k_attractive(-0.1),
    k_repulsive(-0.01),
    gamma(2),
    eta_0(70),
    rho(1.0),
    nh_(nh)

{

}

    /***************************************
    * Artificial Potential Fields function
    ****************************************/

geometry_msgs::Twist apf(const double& k_attractive, const double& k_repulsive,
                         const double& rho, const double& eta_0,
                         std_msgs::Float64MultiArray map_info)
{
    /***************************************************************************
    * Local variables for Artificial Potential Fields formula
    *
    ***************************************************************************/

    double attractive_potential_x;
    double attractive_potential_y;
    double attractive_potential_theta;

	double repulsive_potential_x;
    double repulsive_potential_y;
    double repulsive_potential_theta;

    double eta_i; //distance from obstacle; etai(q);
    double e;     //distance from goal; e(q)

    /******************************************************************
     * conversione dati contenuti nell'array map_info
     * conversione da std_msgs::Float64MultiArray -> Eigen::MatrixXf;
     ******************************************************************/

    int rows = map_info.layout.dim[0].size;
    int cols = map_info.layout.dim[1].size;

    Eigen::MatrixXf obstacles_map = Eigen::Map<Eigen::MatrixXf>(map_info.data, rows, cols);

    /**********************************************************************
     *Attractive Potential
     **********************************************************************/

    /*********************************************************************
     * Set the goal 3 m ahead (static goal, always 3 m from the robot);
     ********************************************************************/
     Eigen::Vector2f goal(cols/2, 3000)
     Eigen::Vector2f rtg(goal.x() - cols/2, goal.y() + 0 ); //Vettore robot -> goal
     e = rtg.norm();


     if (e <= rho)
     {
        /********************************************************************
        * Paraboloidal
        * Linear force in e, robot behavior near the goal
        ********************************************************************/
         attractive_potential_x = k_attractive * rtg.x();
         attractive_potential_y = k_attractive * rtg.y();
     }
     else // if(e > rho)
     {
        /*******************************************************************
        * Conical
        * Constant force, robot behavior far from the goal
        *******************************************************************/
         attractive_potential_x = k_attractive * (rtg.x() / e);
         attractive_potential_y = k_attractive * (rtg.y() / e);
     }


     /**********************************************
     * Repulsive Potential
     ***********************************************/

    for(int x = 0; x < rows; x++){
        for (int y = 0; y < cols; y++) {
            if (obstacles_map(i, j) == 1.0f) {

                /******************************************************
                * Define eta_i: distance between obstacle and MARRtino;
                *******************************************************/

                //Il robot si trova al centro dell'immagine;
                Eigen::Vector2f rto(x - (cols/2), y - 0 ); //Vettore robot -> obstacle
                eta_i = rto.norm();

                /******************************************************
                 *Repulsive potential formula (gradient)
                *******************************************************/

                if (eta_i <= eta_0) {
                    repulsive_potential_x = (k_repulsive/pow(eta_i, 2)) * pow((1/eta_i - 1/eta_0), gamma - 1) * ( rto.x() / eta_i); //Eigen access to Vector: rto(0)
                    repulsive_potential_y = (k_repulsive/pow(eta_i, 2)) * pow((1/eta_i - 1/eta_0), gamma - 1) * ( rto.y() / eta_i); //Eigen access to Vector: rto(1)
                }

                else{ //if(eta_i > eta_0)
                    repulsive_potential_x = 0.0;
                    repulsive_potential_y = 0.0;
                }

                repulsive_potential_theta = repulsive_potential_x * (yb_corner[i].getOrigin().x()*sin(tf_yb_origin_yaw)+yb_corner[i].getOrigin().y()*cos(tf_yb_origin_yaw))

                + d_u_rep_y * (yb_corner[i].getOrigin().x()*cos(tf_yb_origin_yaw)-yb_corner[i].getOrigin().y()*sin(tf_yb_origin_yaw));

                //Sommatoria di tutte le forze repulsive agenti sulle coordinate
                vel.linear.x  += repulsive_potential_x;
			    vel.linear.y  += repulsive_potential_y;
	            vel.angular.z += repulsive_potential_theta;

            }
        }
    }

    //Sommatoria di tutte le forze attrattive agenti sulle coordinate
    vel.linear.x  += attractive_potential_x;
    vel.linear.y  += attractive_potential_y;
    vel.angular.z += attractive_potential_theta;

    return vel;
}


void apf_motion_planner::init()
{
    pub_velocity_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    sub_odom_ = nh_.subscribe;

    sub_obstacle_mapper_ = nh_.subscribe<std_msgs::Float64MultiArray>("/camera/obstacles_mapper_2d");
}
