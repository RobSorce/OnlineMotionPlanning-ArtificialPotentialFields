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

    k_attractive(0.1),
    k_repulsive(0.01),
    k_theta(0.1),
    gamma(2),
    eta_0(70),
    rho(1.0),
    nh_(nh)

{
        std::cerr << "/* error message Constructor*/" << '\n';
}

    /***************************************
    * Artificial Potential Fields function
    ****************************************/

geometry_msgs::Twist apf_motion_planner::apf(const Eigen::MatrixXf& map_info)
{
//    std::cerr << "/* error message apf start*/" << '\n';
    /***************************************************************************
    * Local variables for Artificial Potential Fields formula
    *
    ***************************************************************************/
    geometry_msgs::Twist vel;

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

    int rows = map_info.rows();//; map_info.layout.dim[0].size;
    int cols = map_info.cols(); //.layout.dim[1].size;

    //Eigen::MatrixXf obstacles_map = Eigen::Map<Eigen::MatrixXd>(map_info.data, rows, cols).cast<float>();

    /*************************************************************************
     * Set the goal 3 m ahead (static goal, always set to 3 m from the robot);
     ************************************************************************/
     Eigen::Vector2f goal(cols/2, 3000);
     Eigen::Vector2f rtg(goal.x() - (cols/2), goal.y() + 0 ); //Vettore robot -> goal
     e = rtg.norm();

     /**********************************************************************
      *Attractive Potential
      **********************************************************************/

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

     attractive_potential_theta = k_theta * std::atan2(attractive_potential_y, attractive_potential_x);

     /**********************************************
     * Repulsive Potential
     ***********************************************/

    for(int x = 0; x < rows; x++) {
        for (int y = 0; y < cols; y++) {
            if (map_info(x, y) == 1.0f)
            {

                /******************************************************
                * Define eta_i: distance between obstacle and MARRtino;
                *******************************************************/

                //Il robot si trova al centro dell'immagine;
                Eigen::Vector2f rto(x - (cols/2), y - 0 ); //Vettore robot -> obstacle
                eta_i = rto.norm();

                /******************************************************
                 *Repulsive potential formula (gradient)
                *******************************************************/

                if (eta_i <= eta_0)
                {   /*Modificato: tolto elevamento a potenza gamma - 1*/
                    repulsive_potential_x = (k_repulsive/pow(eta_i, 2)) * std::pow((1/eta_i - 1/eta_0), gamma ) * ( rto.x() / eta_i); //Eigen access to Vector: rto(0)
                    repulsive_potential_y = (k_repulsive/pow(eta_i, 2)) * std::pow((1/eta_i - 1/eta_0), gamma ) * ( rto.y() / eta_i); //Eigen access to Vector: rto(1)
                }

                else
                { //if(eta_i > eta_0)
                    repulsive_potential_x = 0.0;
                    repulsive_potential_y = 0.0;
                }

                repulsive_potential_theta = k_theta * std::atan2(repulsive_potential_y, repulsive_potential_x);

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

void apf_motion_planner::apfCallback(const std_msgs::Float64MultiArray::ConstPtr& map_info)
{
    std::cerr << "/* error message apfCallback start*/" << '\n';
    int rows = map_info->layout.dim[0].size;
    int cols = map_info->layout.dim[1].size;

    double* obstacles_array = const_cast<double*>(map_info->data.data());
    Eigen::MatrixXf obstacles_map = Eigen::Map<Eigen::MatrixXd>(obstacles_array, rows, cols).cast<float>();

    vel_ = apf(obstacles_map);
    pub_velocity_.publish(vel_);

    generate_potential_map(obstacles_map);

    std::cerr << "/* error message apfCallback end*/" << '\n';
}

/****************************************************************************
 * Generates and shows the potential field map: shows the direction of the
 * potential fields acting on each pixel of the map using arrows;
 *
 * UNCOMMENT to show the potential field map;
 ****************************************************************************/

void apf_motion_planner::generate_potential_map(const Eigen::MatrixXf& obstacles_map)
{
    std::cerr << "/* error message generate_potential_map start*/" << '\n';
    int rows = obstacles_map.rows() * 10;
    int cols = obstacles_map.cols() * 10;

    geometry_msgs::Twist velocity;

    cv::Mat1b cvMatObs(rows, cols);

    for (int row = 0; row < obstacles_map.rows(); row += 100) {
        for (int col = 0; col < obstacles_map.cols(); col += 100) {

            velocity = apf(obstacles_map);

            /***************************************************************************
             * C++: void arrowedLine(Mat& img, Point pt1, Point pt2, const Scalar& color,
             * int thickness=1, int line_type=8, int shift=0, double tipLength=0.1)
             **************************************************************************/
            cv::arrowedLine(cvMatObs, cv::Point(row * 10, col * 10), cv::Point(row * 10 + velocity.linear.x, col * 10 + velocity.linear.y), cv::Scalar(0, 0, 255), 1, 8, 0, 0.1);
        }
        std::cerr << "/* error message generate_potential_map end loop*/" << '\n';
    }
    std::cerr << "/* error message generate_potential_map creation*/" << '\n';

    cv::imshow("Potential Map", cvMatObs);
    cv::waitKey(30);
    std::cerr << "/* error message generate_potential_map end*/" << '\n';
}


void apf_motion_planner::init()
{
    pub_velocity_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    //sub_odom_ = nh_.subscribe;

    sub_obstacle_mapper_ = nh_.subscribe<std_msgs::Float64MultiArray>("/camera/obstacles2D", 1, &apf_motion_planner::apfCallback, this);
}


/**********************************
*  Ti prego... fai qualcosa!!!!
*********************************/
