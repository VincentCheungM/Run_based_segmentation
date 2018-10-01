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
#include <pcl/common/centroid.h>

#include <velodyne_pointcloud/point_types.h>

//Customed Point Struct for holding clustered points
namespace scan_line_run
{
  /** Euclidean Velodyne coordinate, including intensity and ring number, and label. */
  struct PointXYZIRL
  {
    PCL_ADD_POINT4D;                    // quad-word XYZ
    float    intensity;                 ///< laser intensity reading
    uint16_t ring;                      ///< laser ring number
    uint16_t label;                     ///< point label
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
  } EIGEN_ALIGN16;

}; // namespace scan_line_run

#define SLRPointXYZIRL scan_line_run::PointXYZIRL
#define VPoint velodyne_pointcloud::PointXYZIR
#define RUN pcl::PointCloud<SLRPointXYZIRL>
// Register custom point struct according to PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(scan_line_run::PointXYZIRL,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring)
                                  (uint16_t, label, label))

// using eigen lib
#include <Eigen/Dense>
using Eigen::MatrixXf;
using Eigen::JacobiSVD;
using Eigen::VectorXf;

pcl::PointCloud<VPoint>::Ptr g_seeds_pc(new pcl::PointCloud<VPoint>());
pcl::PointCloud<VPoint>::Ptr g_ground_pc(new pcl::PointCloud<VPoint>());
pcl::PointCloud<VPoint>::Ptr g_not_ground_pc(new pcl::PointCloud<VPoint>());
pcl::PointCloud<SLRPointXYZIRL>::Ptr g_all_pc(new pcl::PointCloud<SLRPointXYZIRL>());


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
    ros::Publisher all_points_pub_;

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
    all_points_pub_ =  node_handle_.advertise<sensor_msgs::PointCloud2>("/all_points", 2);
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
    // Create covarian matrix in single pass.
    // TODO: compare the efficiency.
    Eigen::Matrix3f cov;
    Eigen::Vector4f pc_mean;
    pcl::computeMeanAndCovarianceMatrix(*g_ground_pc, cov, pc_mean);
    // Singular Value Decomposition: SVD
    JacobiSVD<MatrixXf> svd(cov,Eigen::DecompositionOptions::ComputeFullU);
    // use the least singular vector as normal
    normal_ = (svd.matrixU().col(2));
    // mean ground seeds value
    Eigen::Vector3f seeds_mean = pc_mean.head<3>();

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
    pcl::PointCloud<VPoint> laserCloudIn_org;
    pcl::fromROSMsg(*in_cloud_msg, laserCloudIn_org);
    // For mark ground points and hold all points
    SLRPointXYZIRL point;
    for(size_t i=0;i<laserCloudIn.points.size();i++){
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;
        point.intensity = laserCloudIn.points[i].intensity;
        point.ring = laserCloudIn.points[i].ring;
        point.label = 0u;// 0 means uncluster
        g_all_pc->points.push_back(point);
    }
    //std::vector<int> indices;
    //pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn,indices);
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
        MatrixXf points(laserCloudIn_org.points.size(),3);
        int j =0;
        for(auto p:laserCloudIn_org.points){
            points.row(j++)<<p.x,p.y,p.z;
        }
        // ground plane model
        VectorXf result = points*normal_;
        // threshold filter
        for(int r=0;r<result.rows();r++){
            if(result[r]<th_dist_d_){
                g_all_pc->points[r].label = 1u;// means ground
                g_ground_pc->points.push_back(laserCloudIn_org[r]);
            }else{
                g_all_pc->points[r].label = 0u;// means not ground and non clusterred
                g_not_ground_pc->points.push_back(laserCloudIn_org[r]);
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
    // publish all points
    sensor_msgs::PointCloud2 all_points_msg;
    pcl::toROSMsg(*g_all_pc, all_points_msg);
    all_points_msg.header.stamp = in_cloud_msg->header.stamp;
    all_points_msg.header.frame_id = in_cloud_msg->header.frame_id;
    all_points_pub_.publish(all_points_msg);
    g_all_pc->clear();
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "GroundPlaneFit");
    GroundPlaneFit node;
    ros::spin();

    return 0;

}