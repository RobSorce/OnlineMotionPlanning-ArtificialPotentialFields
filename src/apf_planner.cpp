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
#include <limits>

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

    /*########################################################################
     * Nota: Accesso dati a matrice OpenCV
     *       i dati della matrice OpenCV sono immagazzinati in formato
     *       ColMajor, scandisco prima le colonne, successivamente le righe;
     *       x = colonne (width), y = righe (height);
     *
     *########################################################################
    */

apf_motion_planner::apf_motion_planner(ros::NodeHandle& nh, RepulsiveType r_type) :

    k_attractive(0.01),
    k_repulsive(1000.0),
    k_theta(0.1),
    gamma(2),
    eta_0(300),
    rho(1.0),
    r_type(r_type),
    nh_(nh)
{

}

/****************************************************
 * Attractive potential function                    *
*****************************************************/

geometry_msgs::Twist apf_motion_planner::attractive_potential(float xgoal, float ygoal, float xrobot, float yrobot)
{

    geometry_msgs::Twist attractive_vel;

    double attractive_potential_x;
    double attractive_potential_y;
    double attractive_potential_theta;

    double e;     //distance from goal; e(q)

    Eigen::Vector2f rtg(xgoal - xrobot, ygoal - yrobot); //Vettore robot -> goal
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

    //Sommatoria di tutte le forze attrattive agenti sulle coordinate
    attractive_vel.linear.x  = attractive_potential_x;
    attractive_vel.linear.y  = attractive_potential_y;
    attractive_vel.angular.z = attractive_potential_theta;

    ////////////////////////////////////////////////////////////////////////////
    // Print vel data
    //std::cerr << vel << '\n';
    ////////////////////////////////////////////////////////////////////////////

    return attractive_vel;

}

/******************************************************
*Repulsive Potential function                         *
*******************************************************/
geometry_msgs::Twist apf_motion_planner::repulsive_potential(float xr, float yr, float xo, float yo)
{
    geometry_msgs::Twist repulsive_vel;

    double repulsive_potential_x;
    double repulsive_potential_y;
    double repulsive_potential_theta;

    double eta_i; //distance from obstacle; etai(q);

    //Il robot si trova al centro dell'immagine;
    Eigen::Vector2f rto(xo - xr, yo - yr); //Vettore robot -> obstacle
    eta_i = rto.norm();

    /******************************************************
    *Repulsive potential formula (gradient)
    *******************************************************/
    if (eta_i <= eta_0)
    {
        repulsive_potential_x = (k_repulsive/pow(eta_i, 2)) * std::pow((1/eta_i - 1/eta_0), gamma - 1) * ( rto.x() / eta_i); //Eigen access to Vector: rto(0)
        repulsive_potential_y = (k_repulsive/pow(eta_i, 2)) * std::pow((1/eta_i - 1/eta_0), gamma - 1) * ( rto.y() / eta_i); //Eigen access to Vector: rto(1)
    }
    else
    { //if(eta_i > eta_0)
        repulsive_potential_x = 0.0;
        repulsive_potential_y = 0.0;
    }

    repulsive_potential_theta = k_theta * std::atan2(repulsive_potential_y, repulsive_potential_x);

    //Sommatoria di tutte le forze repulsive agenti sulle coordinate

    repulsive_vel.linear.x  -= repulsive_potential_x;
    repulsive_vel.linear.y  -= repulsive_potential_y;
    repulsive_vel.angular.z -= repulsive_potential_theta;

    ////////////////////////////////////////////////////////////////////////////
    // Print vel data
    //std::cerr << vel << '\n';
    ////////////////////////////////////////////////////////////////////////////

    return repulsive_vel;
}


/******************************************************
*Vo rtex Potential function                         *
*******************************************************/
geometry_msgs::Twist apf_motion_planner::vortex_potential(float xr, float yr, float xo, float yo)
{
    geometry_msgs::Twist repulsive_vel;

    double repulsive_potential_x;
    double repulsive_potential_y;
    double repulsive_potential_theta;

    double eta_i; //distance from obstacle; etai(q);

    //Il robot si trova al centro dell'immagine;
    Eigen::Vector2f rto(xo - xr, yo - yr); //Vettore robot -> obstacle
    eta_i = rto.norm();

    /******************************************************
    *Repulsive potential formula (gradient)
    *******************************************************/
    if (eta_i <= eta_0)
    {
        repulsive_potential_x = (k_repulsive/pow(eta_i, 2)) * std::pow((1/eta_i - 1/eta_0), gamma - 1) * ( rto.x() / eta_i); //Eigen access to Vector: rto(0)
        repulsive_potential_y = (k_repulsive/pow(eta_i, 2)) * std::pow((1/eta_i - 1/eta_0), gamma - 1) * ( rto.y() / eta_i); //Eigen access to Vector: rto(1)
    }
    else
    { //if(eta_i > eta_0)
        repulsive_potential_x = 0.0;
        repulsive_potential_y = 0.0;
    }

    repulsive_potential_theta = k_theta * std::atan2(repulsive_potential_x, -repulsive_potential_y);

    //Sommatoria di tutte le forze repulsive agenti sulle coordinate

    repulsive_vel.linear.x  -= -repulsive_potential_y;
    repulsive_vel.linear.y  -=  repulsive_potential_x;
    repulsive_vel.angular.z -=  repulsive_potential_theta;

    ////////////////////////////////////////////////////////////////////////////
    // Print vel data
    //std::cerr << vel << '\n';
    ////////////////////////////////////////////////////////////////////////////

    return repulsive_vel;
}

std::vector<ObstacleInfo> extractObstaclesInfo(const cv::Mat& obstacles_map, int num_obstacles)
{
    std::vector<ObstacleInfo> vec(num_obstacles);

    for (int row = 0; row < obstacles_map.rows; row++) {
        for (int col = 0; col < obstacles_map.cols; col++) {
            if (obstacles_map.at<int>(row, col) != 0)
            {
                vec[obstacles_map.at<int>(row, col)-1].push_back(cv::Point(col, row));
            }
        }
    }

    return vec;
}



geometry_msgs::Twist apf_motion_planner::artificial_potential_fields(const std::vector<ObstacleInfo>& obstacles_array, float xr, float yr)
{
    /***************************************************************************
    * Local variables for Artificial Potential Fields formula
    *
    ***************************************************************************/

    geometry_msgs::Twist attractive_vel;
    geometry_msgs::Twist repulsive_vel;
    geometry_msgs::Twist total_vel;

	double repulsive_potential_x;
    double repulsive_potential_y;
    double repulsive_potential_theta;

    std::vector<cv::Point> obstacle_closest_points(obstacles_array.size());

    Eigen::Vector2f goal(xr, yr + 1000);

    /*******************************************************************
     * chiamata a funzione attractive potential: salvo i dati          *
     * geometry_msgs::Twist nella variabile attractive_vel;            *
     *******************************************************************/
    attractive_vel = attractive_potential(goal.x(), goal.y(), xr, yr);


    int o_idx = 0;
    cv::Point robot_position(xr, yr);

    for(const ObstacleInfo& obstacle : obstacles_array) {
        double old_distance = std::numeric_limits<double>::max(); //std::static_cast<double>((1 << 31) - 1); //  ~MAX_INT

        for(const cv::Point& obstacle_point : obstacle ) {

            double new_distance = cv::norm( cv::Mat(obstacle_point), cv::Mat(robot_position));

            if(new_distance < old_distance) {
                obstacle_closest_points[o_idx] = obstacle_point;
                old_distance = new_distance;
            }
        }
        ++o_idx;
    }

     /*******************************************************
     * Repulsive field on closest points (one for obstacle) *
     *******************************************************/
     geometry_msgs::Twist point_vel;
     for(const cv::Point& obstacle : obstacle_closest_points) {
         switch (r_type) {
            case RepulsiveType::REPULSIVE:
                point_vel = repulsive_potential(goal.x(), goal.y(), obstacle.x, obstacle.y);
                break;
            case RepulsiveType::VORTEX:
                point_vel = vortex_potential(goal.x(), goal.y(), obstacle.x, obstacle.y);
                break;
        }

        repulsive_vel.linear.x  += point_vel.linear.x;
        repulsive_vel.linear.y  += point_vel.linear.y;
        repulsive_vel.angular.z += point_vel.angular.z;
     }

     //Sommatoria di tutte le forze attrattive + repulsive agenti sulle coordinate
     total_vel.linear.x  = repulsive_vel.linear.x  + attractive_vel.linear.x;
     total_vel.linear.y  = repulsive_vel.linear.y  + attractive_vel.linear.y;
     total_vel.angular.z = repulsive_vel.angular.z + attractive_vel.angular.z;

     ///////////////////////////////////////////////////////////////////////////
     //Print vel data
     //std::cerr << vel << '\n';
     ///////////////////////////////////////////////////////////////////////////

     return total_vel;
}




void apf_motion_planner::apfCallback(const std_msgs::Float64MultiArray::ConstPtr& map_info)
{
    cv::Mat obstacles_map;         //matrice degli ostacoli dove salvo i dati std_msgs::Float64MultiArray;
    cv::Mat labeled_obstacles_map; //matrice degli ostacoli labeled;

    int num_obstacles;
    int rows = map_info->layout.dim[0].size;
    int cols = map_info->layout.dim[1].size;

    obstacles_map = cv::Mat1d(rows, cols, const_cast<double*>(map_info->data.data())); //genero la matrice degli ostacoli OpenCV <- map_info
    std::cerr << "/* error message1 */" << '\n';
    cv::Mat converted2, converted;
    std::cerr << "/* error message2 */" << '\n';
    obstacles_map.convertTo(converted2, CV_8U);
    std::cerr << "/* error message3 */" << '\n';
    cv::threshold(converted2, converted, 1, 1, cv::THRESH_BINARY);
    std::cerr << "/* error message4 */" << '\n';
    //obstacles_map.convertTo(converted, CV_8U);
    //obstacles_map.convertTo()

    num_obstacles = cv::connectedComponents(converted, labeled_obstacles_map, 8, CV_32S);   //ritorna il numero di ostacoli
    std::cerr << "/* error message  connectedComponents*/" << '\n';
    std::vector<ObstacleInfo> obstacles = extractObstaclesInfo(labeled_obstacles_map, num_obstacles); //std::vector<std::vector<cv::Point>>
    std::cerr << "/* error message 5*/" << '\n';
    vel_ = artificial_potential_fields(obstacles, cols/2, 0);

    std::cerr << "/* error message 6*/" << '\n';
    //vel_ = vortex(obstacles_map, cols/2.0, 0);
    pub_velocity_.publish(vel_);

    std::cerr << "/* error message 7*/" << '\n';
    generate_potential_map(labeled_obstacles_map);

    std::cerr << "/* error message end */" << '\n';
}

    /**************************************************************************
     * Generates and shows the potential field map: shows the direction of the
     * potential fields acting on each pixel of the map using arrows;
     *
     * UNCOMMENT to show the potential field map;
     **************************************************************************/

void apf_motion_planner::generate_potential_map(const cv::Mat1f& obstacles_map)
{
    std::cerr << "/* error message map*/" << '\n';
    geometry_msgs::Twist velocity;
    cv::Mat potential_map = obstacles_map;
    std::cerr << "/* error message map 2*/" << '\n';
    for (int y = 0; y < obstacles_map.rows; y += 100) {
        for (int x = 0; x < obstacles_map.cols; x += 100) {
std::cerr << "/* error message map loop*/" << '\n';
            velocity = artificial_potential_fields(obstacles_map, x, y);
std::cerr << "/* error message map loop end*/" << '\n';
            /***************************************************************************
             * C++: void arrowedLine(Mat& img, Point pt1, Point pt2, const Scalar& color,
             * int thickness=1, int line_type=8, int shift=0, double tipLength=0.1)
             **************************************************************************/
            cv::arrowedLine(potential_map, cv::Point(x, y), cv::Point(x + velocity.linear.x*1500, y + velocity.linear.y*1500), cv::Scalar(255, 255, 255), 1, 1, 0, 0.1);
            //cv::arrowedLine(potential_map, cv::Point(x, y), cv::Point(x + velocity.linear.x*1500, y + velocity.linear.y*1500)/100, cv::Scalar(255, 255, 255), 1, 1, 0, 0.1);
std::cerr << "/* error message map END */" << '\n';
        }
    }

    cv::imshow("Potential Map", potential_map);
    cv::waitKey(30);

}

/********************************************************************************
TODO
void apf_motion_planner::odomCallback(const nav_msgs::Odometry latest_odom) {

    tf::Pose pose;
	tf::poseMsgToTF(latest_odom.pose.pose, pose);
	theta = tf::getYaw(pose.getRotation()) * 180 / PI;

	//ROS_INFO("Relative rotation: %f", theta);
}
*******************************************************************************/

/*******************************************************************
TODO
void apf_motion_planner::get_MARRtino_pose(tf::TransformListener* transform_listener))
{

}
*************************************************************************/

void apf_motion_planner::init()
{
    pub_velocity_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    //sub_odom_ = nh_.subscribe<nav_msgs::Odometry>("/odom", 100, apf_motion_planner::odomCallback);

    sub_obstacle_mapper_ = nh_.subscribe<std_msgs::Float64MultiArray>("/camera/obstacles2D", 1, &apf_motion_planner::apfCallback, this);
}
