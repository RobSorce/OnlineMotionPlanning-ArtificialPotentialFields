/*
 * obstacles_mapper_2d Node:
 * Subscribes to /camera/depth/points topic to gather PointCloud2 messages
 * (Obtain a pointcloud from the Microsoft Kinect sensor), converts
 * PointCloud2 messages into std_msgs::Float64MultiArray and publishes data
 * on another topic /camera/obstacles2D;
 *
 * messages from that topic will be read by apf_planner node
 * in order to compute artificial potential fields around obstacles and properly
 * set a goal to be reached;
 */

#include <../include/obstacles_mapper_2d.h>

    /*#########################################################################
     * Inizializzazione lista parametri del costruttore
     * Nota: dichiarazione parametri esattamente nell'ordine in cui sono stati
     *       definiti nella classe; Altrimenti genera errore a run-time;
     *
     * Caveat: Inizializzazione lista atttributi della classe, piuttosto che
     *         dentro il corpo del costruttore, così da evitare ulteriore
     *         inizializzazione di default ridondante;
     *#########################################################################
    */

obstacles_mapper_2d::obstacles_mapper_2d(ros::NodeHandle& nh) :

    cell_dimension_mm(20),
    desired_depth_mm(4000),
    desired_width_mm(10000),
    nh_(nh)

{

}

void obstacles_mapper_2d::pclCallback(const sensor_msgs::PointCloud2::ConstPtr& pc)
{

	ros::Time time_stamp = ros::Time::now();

    if (listener_.canTransform("base_link", "camera_depth_optical_frame", time_stamp, NULL)) {
        listener_.waitForTransform("base_link", "camera_depth_optical_frame", time_stamp, ros::Duration(1e-3));
        listener_.lookupTransform("base_link", "camera_depth_optical_frame", time_stamp, this->point_cloud_tranform_);
    }


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pt           (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rotated      (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_mid (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_up  (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*pc, *cloud_pt);

    /*
    * Applico un filtro sui punti della PointCloud che vedo come ostacoli,
    * Il filtraggio è applicato sulla coordinata y, altezza della kinect.
    * Nota: La coordinata y della kinect è rivolta verso il basso.
    *
    * Caveat: nel filtro PassThrough(setFilterLimits), essendo la y rivolta
    *         verso il basso, e applicando il filtro prima della trasformazione
    *         del frame, i valori del filtro sono passati al contrario (+, -);
    *
    **/

    // Executing the transformation
    pcl_ros::transformPointCloud("base_link", *cloud_pt, *cloud_rotated, listener_);

    // Create the filtering object z
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud_rotated);
    pass.setFilterFieldName ("z");         //coordinata altezza della kinect (frame originale, y down)
    pass.setFilterLimits (-0.025, +0.025); //filtering the points between the given range;
    pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_filtered_mid);

    /*
    * Applico un secondo filtro sull'asse y(altezza del frame kinect)
    * Filtro tutti gli ostacoli di altezza > Robot: Non sono ostacoli!
    * Riesco a passarci sotto.
    **/

    // Create the filtering object z
    pcl::PassThrough<pcl::PointXYZ> pass2;
    pass2.setInputCloud (cloud_filtered_mid);
    pass2.setFilterFieldName ("z");         //coordinata altezza della kinect (frame originale, y down)
    pass2.setFilterLimits (+0.025, +0.25);     //filtering the points between the given range;
    pass2.setFilterLimitsNegative (false);
    pass2.filter (*cloud_filtered_up);

    /*
    * Eseguo la trasformazione delle coordinate, dal frame della
    * kinect(camera/depth/point) (left, down, forward) (x, y, z) -> mie coordinate desiderate.
    * left, up, forward(x, y, z)
    *
    */

    //
    // tf::transformTFToEigen(pcl_transform, eigen_transform);
    // pcl::transformPointCloud (*cloud_filtered_up, *cloud_rotated, eigen_transform);

/**************************************
 * creating obstacle's matrix         *
 *************************************/
    int side_sight_mm = 5000;

    //map size parameters (computed based on resolutions and sight)
    int cols = desired_width_mm / cell_dimension_mm; //1000;
    int rows = desired_depth_mm / cell_dimension_mm; //400;

    //int minxc = 600;
    //int minzc = 600;

    Eigen::MatrixXf mat = Eigen::MatrixXf::Zero(rows, cols);
    for (const pcl::PointXYZ& point : *cloud_filtered_up) {

        int yc = static_cast<int>(point.y * 1000) + side_sight_mm; // conversione in mm
        int xc = static_cast<int>(point.x * 1000); // mm

        //convert in cell indices
        yc = yc / cell_dimension_mm;
        xc = xc / cell_dimension_mm;
        //minxc = std::min(minxc, xc);
        //minzc = std::min(minzc, zc);

        //check bounds

        if (yc >= cols || xc >= rows)
            continue;
        if (yc < 0 || xc < 0)
            continue;

        mat(xc, yc) = 1.0f;
    }

    //Visualize obstacle matrix;
    //UNCOMMENT to visualize the output obstacle map;
    ///////////////////////////////////////////////////////////////////////////
    // cv::Mat1b cvMat(rows, cols);
    // cv::eigen2cv(mat, cvMat);
    // cv::imshow("Obstacle Map", cvMat);
    // cv::waitKey(30);
    ///////////////////////////////////////////////////////////////////////////

    // conversione matrice da MatrixXf -> std_msgs::Float64MultiArray
    std_msgs::Float64MultiArray map_info;

    tf::matrixEigenToMsg(mat, map_info);
    pub_.publish(map_info);

    /**
    * PCL Viewer
    *
    *
    *viewer_filtered->removeAllPointClouds();
    *viewer_filtered->addPointCloud<pcl::PointXYZ> (cloud_voxelized, "Filtered" );
    *viewer_filtered->spinOnce();
    */

}

void obstacles_mapper_2d::init()
{
    pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/camera/obstacles2D", 1);

    sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("camera/depth/points", 1, &obstacles_mapper_2d::pclCallback, this);
}
