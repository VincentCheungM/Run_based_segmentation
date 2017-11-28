/*
    @file groundplanfit.cpp
    @brief ROS Node for ground plane fitting

    This is a ROS node to perform ground plan fitting.
    Implementation accoriding to <Fast Segmentation of 3D Point Clouds: A Paradigm>

    In this case, it's assumed that the x,y axis points at sea-level,
    and z-axis points up. The sort of height is based on the Z-axis value.

    @author Vincent Cheung(VincentCheungm)
    @bug Sometimes the plane is not fit.
*/

#include <iostream>
// For disable PCL complile lib, to use PointXYZIR    
#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <velodyne_pointcloud/point_types.h>
#define VPoint velodyne_pointcloud::PointXYZIR
// using eigen lib
#include <Eigen/Dense>
using Eigen::MatrixXf;
using Eigen::JacobiSVD;
using Eigen::VectorXf;

pcl::PointCloud<VPoint>::Ptr g_seeds_pc(new pcl::PointCloud<VPoint>());
pcl::PointCloud<VPoint>::Ptr g_ground_pc(new pcl::PointCloud<VPoint>());
pcl::PointCloud<VPoint>::Ptr g_not_ground_pc(new pcl::PointCloud<VPoint>());

/*
    @brief Compare function to sort points. Here use z axis.
    @return z-axis accent
*/
bool point_cmp(VPoint a, VPoint b){
    return a.z<b.z;
}

/*
    @brief Ground Plane fitting ROS Node.
    @param Velodyne Pointcloud topic.
    @param Sensor Model.
    @param Sensor height for filtering error mirror points.
    @param Num of segment, iteration, LPR
    @param Threshold of seeds distance, and ground plane distance
    
    @subscirbe:/velodyne_points
    @publish:/points_no_ground, /points_ground
*/
class GroundPlaneFit{
public:
    GroundPlaneFit();
private:
    ros::NodeHandle node_handle_;
    ros::Subscriber points_node_sub_;
    ros::Publisher ground_points_pub_;
    ros::Publisher groundless_points_pub_;

    std::string point_topic_;

    int sensor_model_;
    double sensor_height_;
    int num_seg_;
    int num_iter_;
    int num_lpr_;
    double th_seeds_;
    double th_dist_;


    void velodyne_callback_(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);
    void estimate_plane_(void);
    void extract_initial_seeds_(const pcl::PointCloud<VPoint>& p_sorted);

    // Model parameter for ground plane fitting
    // The ground plane model is: ax+by+cz+d=0
    // Here normal:=[a,b,c], d=d
    // th_dist_d_ = threshold_dist - d 
    float d_;
    MatrixXf normal_;
    float th_dist_d_;
};    

/*
    @brief Constructor of GPF Node.
    @return void
*/
GroundPlaneFit::GroundPlaneFit():node_handle_("~"){
    // Init ROS related
    ROS_INFO("Inititalizing Ground Plane Fitter...");
    node_handle_.param<std::string>("point_topic", point_topic_, "/velodyne_points");
    ROS_INFO("Input Point Cloud: %s", point_topic_.c_str());

    node_handle_.param("sensor_model", sensor_model_, 32);
    ROS_INFO("Sensor Model: %d", sensor_model_);

    node_handle_.param("sensor_height", sensor_height_, 2.5);
    ROS_INFO("Sensor Height: %f", sensor_height_);

    node_handle_.param("num_seg", num_seg_, 1);
    ROS_INFO("Num of Segments: %d", num_seg_);

    node_handle_.param("num_iter", num_iter_, 3);
    ROS_INFO("Num of Iteration: %d", num_iter_);

    node_handle_.param("num_lpr", num_lpr_, 20);
    ROS_INFO("Num of LPR: %d", num_lpr_);

    node_handle_.param("th_seeds", th_seeds_, 1.2);
    ROS_INFO("Seeds Threshold: %f", th_seeds_);

    node_handle_.param("th_dist", th_dist_, 0.3);
    ROS_INFO("Distance Threshold: %f", th_dist_);

    // Listen to velodyne topic
    points_node_sub_ = node_handle_.subscribe(point_topic_, 2, &GroundPlaneFit::velodyne_callback_, this);
    
    // Publish Init
    std::string no_ground_topic, ground_topic;
    node_handle_.param<std::string>("no_ground_point_topic", no_ground_topic, "/points_no_ground");
    ROS_INFO("No Ground Output Point Cloud: %s", no_ground_topic.c_str());
    node_handle_.param<std::string>("ground_point_topic", ground_topic, "/points_ground");
    ROS_INFO("Only Ground Output Point Cloud: %s", ground_topic.c_str());
    groundless_points_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>(no_ground_topic, 2);
    ground_points_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>(ground_topic, 2);
 
}

/*
    @brief The function to estimate plane model. The
    model parameter `normal_` and `d_`, and `th_dist_d_`
    is set here.
    The main step is performed SVD(UAV) on covariance matrix.
    Taking the sigular vector in U matrix according to the smallest
    sigular value in A, as the `normal_`. `d_` is then calculated 
    according to mean ground points.

    @param g_ground_pc:global ground pointcloud ptr.
    
*/
void GroundPlaneFit::estimate_plane_(void){
    // Create covarian matrix.
    // 1. calculate (x,y,z) mean
    float x_mean = 0, y_mean = 0, z_mean = 0;
    for(int i=0;i<g_ground_pc->points.size();i++){
        x_mean += g_ground_pc->points[i].x;
        y_mean += g_ground_pc->points[i].y;
        z_mean += g_ground_pc->points[i].z;
    }
    // incase of divide zero
    int size = g_ground_pc->points.size()!=0?g_ground_pc->points.size():1;
    x_mean /= size;
    y_mean /= size;
    z_mean /= size;
    // 2. calculate covariance
    // cov(x,x), cov(y,y), cov(z,z)
    // cov(x,y), cov(x,z), cov(y,z)
    float xx = 0, yy = 0, zz = 0;
    float xy = 0, xz = 0, yz = 0;
    for(int i=0;i<g_ground_pc->points.size();i++){
        xx += (g_ground_pc->points[i].x-x_mean)*(g_ground_pc->points[i].x-x_mean);
        xy += (g_ground_pc->points[i].x-x_mean)*(g_ground_pc->points[i].y-y_mean);
        xz += (g_ground_pc->points[i].x-x_mean)*(g_ground_pc->points[i].z-z_mean);
        yy += (g_ground_pc->points[i].y-y_mean)*(g_ground_pc->points[i].y-y_mean);
        yz += (g_ground_pc->points[i].y-y_mean)*(g_ground_pc->points[i].z-z_mean);
        zz += (g_ground_pc->points[i].z-z_mean)*(g_ground_pc->points[i].z-z_mean);
    }
    // 3. setup covarian matrix cov
    MatrixXf cov(3,3);
    cov << xx,xy,xz,
           xy, yy, yz,
           xz, yz, zz;
    cov /= size;
    // Singular Value Decomposition: SVD
    JacobiSVD<MatrixXf> svd(cov,Eigen::DecompositionOptions::ComputeFullU);
    // use the least singular vector as normal
    normal_ = (svd.matrixU().col(2));
    // mean ground seeds value
    MatrixXf seeds_mean(3,1);
    seeds_mean<<x_mean,y_mean,z_mean;
    // according to normal.T*[x,y,z] = -d
    d_ = -(normal_.transpose()*seeds_mean)(0,0);
    // set distance threhold to `th_dist - d`
    th_dist_d_ = th_dist_ - d_;
 
    // return the equation parameters
}


/*
    @brief Extract initial seeds of the given pointcloud sorted segment.
    This function filter ground seeds points accoring to heigt.
    This function will set the `g_ground_pc` to `g_seed_pc`.
    @param p_sorted: sorted pointcloud
    
    @param ::num_lpr_: num of LPR points
    @param ::th_seeds_: threshold distance of seeds
    @param ::

*/
void GroundPlaneFit::extract_initial_seeds_(const pcl::PointCloud<VPoint>& p_sorted){
    // LPR is the mean of low point representative
    double sum = 0;
    int cnt = 0;
    // Calculate the mean height value.
    for(int i=0;i<p_sorted.points.size() && cnt<num_lpr_;i++){
        sum += p_sorted.points[i].z;
        cnt++;
    }
    double lpr_height = cnt!=0?sum/cnt:0;// in case divide by 0
    g_seeds_pc->clear();
    // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
    for(int i=0;i<p_sorted.points.size();i++){
        if(p_sorted.points[i].z < lpr_height + th_seeds_){
            g_seeds_pc->points.push_back(p_sorted.points[i]);
        }
    }
    // return seeds points
}

/*
    @brief Velodyne pointcloud callback function. The main GPF pipeline is here.
    PointCloud SensorMsg -> Pointcloud -> z-value sorted Pointcloud
    ->error points removal -> extract ground seeds -> ground plane fit mainloop
*/
void GroundPlaneFit::velodyne_callback_(const sensor_msgs::PointCloud2ConstPtr& in_cloud_msg){
    // 1.Msg to pointcloud
    pcl::PointCloud<VPoint> laserCloudIn;
    pcl::fromROSMsg(*in_cloud_msg, laserCloudIn);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn,indices);
    // 2.Sort on Z-axis value.
    sort(laserCloudIn.points.begin(),laserCloudIn.end(),point_cmp);
    // 3.Error point removal
    // As there are some error mirror reflection under the ground, 
    // here regardless point under 2* sensor_height
    // Sort point according to height, here uses z-axis in default
    pcl::PointCloud<VPoint>::iterator it = laserCloudIn.points.begin();
    for(int i=0;i<laserCloudIn.points.size();i++){
        if(laserCloudIn.points[i].z < -1.5*sensor_height_){
            it++;
        }else{
            break;
        }
    }
    laserCloudIn.points.erase(laserCloudIn.points.begin(),it);
    // 4. Extract init ground seeds.
    extract_initial_seeds_(laserCloudIn);
    g_ground_pc = g_seeds_pc;
    
    // 5. Ground plane fitter mainloop
    for(int i=0;i<num_iter_;i++){
        estimate_plane_();
        g_ground_pc->clear();
        g_not_ground_pc->clear();

        //pointcloud to matrix
        MatrixXf points(laserCloudIn.points.size(),3);
        int j =0;
        for(auto p:laserCloudIn.points){
            points.row(j++)<<p.x,p.y,p.z;
        }
        // ground plane model
        VectorXf result = points*normal_;
        // threshold filter
        for(int r=0;r<result.rows();r++){
            if(result[r]<th_dist_d_){
                g_ground_pc->points.push_back(laserCloudIn[r]);
            }else{
                g_not_ground_pc->points.push_back(laserCloudIn[r]);
            }
        }
    }

    // publish ground points
    sensor_msgs::PointCloud2 ground_msg;
    pcl::toROSMsg(*g_ground_pc, ground_msg);
    ground_msg.header.stamp = in_cloud_msg->header.stamp;
    ground_msg.header.frame_id = in_cloud_msg->header.frame_id;
    ground_points_pub_.publish(ground_msg);
    // publish not ground points
    sensor_msgs::PointCloud2 groundless_msg;
    pcl::toROSMsg(*g_not_ground_pc, groundless_msg);
    groundless_msg.header.stamp = in_cloud_msg->header.stamp;
    groundless_msg.header.frame_id = in_cloud_msg->header.frame_id;
    groundless_points_pub_.publish(groundless_msg);
    
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "GroundPlaneFit");
    GroundPlaneFit node;
    ros::spin();

    return 0;

}