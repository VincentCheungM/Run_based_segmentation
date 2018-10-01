/*
    @file scanlinerun.cpp
    @brief ROS Node for scan line run

    This is a ROS node to perform scan line run clustring.
    Implementation accoriding to <Fast Segmentation of 3D Point Clouds: A Paradigm>

    In this case, it's assumed that the x,y axis points at sea-level,
    and z-axis points up. The sort of height is based on the Z-axis value.

    @author Vincent Cheung(VincentCheungm)
    @bug .
*/
#define IO
#include <iostream>
#include <forward_list>
// For disable PCL complile lib, to use PointXYZIR, and customized pointcloud    
#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <velodyne_pointcloud/point_types.h>


#ifdef MARKER
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#endif

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

#ifdef IO
#include <pcl/io/pcd_io.h>
#include <boost/format.hpp>
#endif

using namespace std;

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

#define dist(a,b) sqrt(((a).x-(b).x)*((a).x-(b).x)+((a).y-(b).y)*((a).y-(b).y))

/*
    @brief Scan Line Run ROS Node.
    @param Velodyne Pointcloud Non Ground topic.
    @param Sensor Model.
    @param Threshold between points belong to the same run
    @param Threshold between runs
    
    @subscirbe:/all_points
    @publish:/slr
*/
class ScanLineRun{
public:
    ScanLineRun();
private:
    ros::NodeHandle node_handle_;
    ros::Subscriber points_node_sub_;
    ros::Publisher cluster_points_pub_;
    ros::Publisher ring_points_pub_;

    std::string point_topic_;

    int sensor_model_;// also means number of sensor scan line.
    double th_run_;// thresold of distance of points belong to the same run.
    double th_merge_;// threshold of distance of runs to be merged.

    // For organization of points.
    std::vector<std::vector<SLRPointXYZIRL> > laser_frame_;
    std::vector<SLRPointXYZIRL> laser_row_;
    
    std::vector<std::forward_list<SLRPointXYZIRL*> > runs_;// For holding all runs.
    uint16_t max_label_;// max run labels, for disinguish different runs.
    std::vector<std::vector<int> > ng_idx_;// non ground point index.

    // Call back funtion.
    void velodyne_callback_(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);
    // For finding runs on a scanline.
    void find_runs_(int scanline);
    // For update points cluster label after merge action.
    void update_labels_(int scanline);
    // For merge `current` run to `target` run.
    void merge_runs_(uint16_t cur_label, uint16_t target_label);
    
    /// @deprecated methods for smart index
    // Smart idx according to paper, but not useful in my case.
    int smart_idx_(int local_idx, int n_i, int n_j, bool inverse);

#ifdef MARKER
    // For display markers only, however, currently the orientation is bad.
    ros::Publisher marker_array_pub_;
#endif
    // Dummy object to occupy idx 0.
    std::forward_list<SLRPointXYZIRL*> dummy_;

    
#ifdef INTEREST_ONLY
    // For showing interest and index point
    int interest_line_;
    int interest_idx_;
#endif
};    

/*
    @brief Constructor of SLR Node.
    @return void
*/
ScanLineRun::ScanLineRun():node_handle_("~"){
    // Init ROS related
    ROS_INFO("Inititalizing Scan Line Run Cluster...");
    node_handle_.param<std::string>("point_topic", point_topic_, "/all_points");
    ROS_INFO("point_topic: %s", point_topic_.c_str());

    node_handle_.param("sensor_model", sensor_model_, 32);
    ROS_INFO("Sensor Model: %d", sensor_model_);
    
    // Init Ptrs with vectors
    for(int i=0;i<sensor_model_;i++){
        std::vector<int> dummy_vec;
        ng_idx_.push_back(dummy_vec);
    }

    // Init LiDAR frames with vectors and points
    SLRPointXYZIRL p_dummy;
    p_dummy.intensity = -1;// Means unoccupy by any points
    laser_row_ = std::vector<SLRPointXYZIRL>(2251, p_dummy);
    laser_frame_ = std::vector<std::vector<SLRPointXYZIRL> >(32, laser_row_);
    
    // Init runs, idx 0 for interest point, and idx for ground points
    max_label_ = 1;
    runs_.push_back(dummy_);
    runs_.push_back(dummy_);

    node_handle_.param("th_run", th_run_, 0.15);
    ROS_INFO("Point-to-Run Threshold: %f", th_run_);

    node_handle_.param("th_merge", th_merge_, 0.5);
    ROS_INFO("RUN-to-RUN Distance Threshold: %f", th_merge_);

    // Subscriber to velodyne topic
    points_node_sub_ = node_handle_.subscribe(point_topic_, 2, &ScanLineRun::velodyne_callback_, this);
    
    // Publisher Init
    std::string cluster_topic;
    node_handle_.param<std::string>("cluster", cluster_topic, "/slr");
    ROS_INFO("Cluster Output Point Cloud: %s", cluster_topic.c_str());   
    cluster_points_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2 >(cluster_topic, 10);
    
#ifdef MARKER
    // Publisher for markers.
    ROS_INFO("Publishing jsk markers at: %s", "cluster_ma");
    marker_array_pub_ = node_handle_.advertise<visualization_msgs::MarkerArray>("cluster_ma", 10);
#endif

    
#ifdef INTEREST_ONLY
    // For showing interest point and its neighbour points only.
    ROS_INFO("Showing interested points only, starts from line: %d idx: %d", interest_line_, interest_idx_);
    node_handle_.param("line", interest_line_, 16);
    node_handle_.param("idx", interest_idx_, 500);
#endif
}


/*
    @brief Read points from the given scan_line. 
    The distance of two continuous points will be labelled to the same run. 
    Clusterred points(`Runs`) stored in `runs_[cluster_id]`. 
    
    @param scan_line: The scan line to find runs.
    @return void
*/
void ScanLineRun::find_runs_(int scan_line){
    // If there is no non-ground points of current scanline, skip.
    int point_size = ng_idx_[scan_line].size();
    if(point_size<=0)
        return;

    int non_g_pt_idx = ng_idx_[scan_line][0]; // The first non ground point
    int non_g_pt_idx_l = ng_idx_[scan_line][point_size - 1]; // The last non ground point

    /* Iterate all non-ground points, and compute and compare the distance 
    of each two continous points. At least two non-ground points are needed.
    */
    for(int i_idx=0;i_idx<point_size-1;i_idx++){
        int i = ng_idx_[scan_line][i_idx];
        int i1 = ng_idx_[scan_line][i_idx+1];

        if(i_idx == 0){
            // The first point, make a new run.
            auto &p_0 = laser_frame_[scan_line][i];
            max_label_ += 1;
            runs_.push_back(dummy_);
            laser_frame_[scan_line][i].label = max_label_;
            runs_[p_0.label].insert_after(runs_[p_0.label].cbefore_begin(), &laser_frame_[scan_line][i]);

            if(p_0.label == 0)
                ROS_ERROR("p_0.label == 0");
        }

        // Compare with the next point
        auto &p_i = laser_frame_[scan_line][i];
        auto &p_i1 = laser_frame_[scan_line][i1];

        // If next point is ground point, skip.
        if(p_i1.label == 1u){
            // Add to ground run `runs_[1]`
            runs_[p_i1.label].insert_after(runs_[p_i1.label].cbefore_begin(), &laser_frame_[scan_line][i1]);
            continue;
        }

        /* If cur point is not ground and next point is within threshold, 
        then make it the same run.
           Else, to make a new run.
        */
        if(p_i.label != 1u && dist(p_i,p_i1) < th_run_){
            p_i1.label = p_i.label; 
        }else{
            max_label_ += 1;
            p_i1.label = max_label_;
            runs_.push_back(dummy_);
        }

        // Insert the index.
        runs_[p_i1.label].insert_after(runs_[p_i1.label].cbefore_begin(), &laser_frame_[scan_line][i1]);
        
        if(p_i1.label == 0)
            ROS_ERROR("p_i1.label == 0");    
    }
    
    // Compare the last point and the first point, for laser scans is a ring.
    if(point_size>1){
        auto &p_0 = laser_frame_[scan_line][non_g_pt_idx];
        auto &p_l = laser_frame_[scan_line][non_g_pt_idx_l];

        // Skip, if one of the start point or the last point is ground point.
        if(p_0.label == 1u || p_l.label == 1u){
            return ;
        }else if(dist(p_0,p_l) < th_run_){
            if(p_0.label==0){
                ROS_ERROR("Ring Merge to 0 label");
            }
            /// If next point is within threshold, then merge it into the same run.
            merge_runs_(p_l.label, p_0.label);
        }
    }else if(point_size == 1){
            // The only point, make a new run.
            auto &p_0 = laser_frame_[scan_line][non_g_pt_idx];
            max_label_ += 1;
            runs_.push_back(dummy_);
            laser_frame_[scan_line][non_g_pt_idx].label = max_label_;
            runs_[p_0.label].insert_after(runs_[p_0.label].cbefore_begin(), &laser_frame_[scan_line][non_g_pt_idx]);
    }
    
}


/*
    @brief Update label between points and their smart `neighbour` point
    above `scan_line`.

    @param scan_line: The current scan line number.
*/
void ScanLineRun::update_labels_(int scan_line){
    // Iterate each point of this scan line to update the labels.
    int point_size_j_idx = ng_idx_[scan_line].size();
    // Current scan line is emtpy, do nothing.
    if(point_size_j_idx==0) return;

    // Iterate each point of this scan line to update the labels.
    for(int j_idx=0;j_idx<point_size_j_idx;j_idx++){
        int j = ng_idx_[scan_line][j_idx];

        auto &p_j = laser_frame_[scan_line][j];

        // Runs above from scan line 0 to scan_line
        for(int l=scan_line-1;l>=0;l--){
            if(ng_idx_[l].size()==0)
                continue;

            // Smart index for the near enough point, after re-organized these points.
            int nn_idx = j;

            if(laser_frame_[l][nn_idx].intensity ==-1 || laser_frame_[l][nn_idx].label == 1u){
                continue;
            }

            // Nearest neighbour point
            auto &p_nn = laser_frame_[l][nn_idx];
            // Skip, if these two points already belong to the same run.
            if(p_j.label == p_nn.label){
                continue;
            }
            double dist_min = dist(p_j, p_nn);

            /* Otherwise,
            If the distance of the `nearest point` is within `th_merge_`, 
            then merge to the smaller run.
            */
            if(dist_min < th_merge_){
                uint16_t  cur_label = 0, target_label = 0;

                if(p_j.label ==0 || p_nn.label==0){
                    ROS_ERROR("p_j.label:%u, p_nn.label:%u", p_j.label, p_nn.label);
                }
                // Merge to a smaller label cluster
                if(p_j.label > p_nn.label){
                    cur_label = p_j.label;
                    target_label = p_nn.label;
                }else{
                    cur_label = p_nn.label;
                    target_label = p_j.label;
                }

                // Merge these two runs.
                merge_runs_(cur_label, target_label);
            }
        }
    }

}

/*
    @brief Merge current run to the target run.

    @param cur_label: The run label of current run.
    @param target_label: The run label of target run.
*/
void ScanLineRun::merge_runs_(uint16_t cur_label, uint16_t target_label){
    if(cur_label ==0||target_label==0){
        ROS_ERROR("Error merging runs cur_label:%u target_label:%u", cur_label, target_label);
    }
    // First, modify the label of current run.
    for(auto &p:runs_[cur_label]){
        p->label = target_label;
    }
    // Then, insert points of current run into target run.
    runs_[target_label].insert_after(runs_[target_label].cbefore_begin(), runs_[cur_label].begin(),runs_[cur_label].end() );
    runs_[cur_label].clear();
}

/*
    @brief Smart index for nearest neighbour on scanline `i` and scanline `j`.

    @param local_idx: The local index of point on current scanline.
    @param n_i: The number of points on scanline `i`.
    @param n_j: The number of points on scanline `j`.
    @param inverse: If true, means `local_idx` is on the outsider ring `j`.
    Otherwise, it's on the insider ring `i`.
    
    @return The smart index.
*/
[[deprecated("Not useful in my case.")]] 
int ScanLineRun::smart_idx_(int local_idx, int n_i, int n_j, bool inverse=false){
    if(inverse==false){
        // In case of zero-divide.
        if(n_i == 0 ) return 0;
        float rate = (n_j*1.0f)/n_i;
        int idx = floor(rate*local_idx);

        // In case of overflow
        if(idx>n_j){
            idx = n_j>1?n_j-1:0;
        }
        return idx;
    }else{
        // In case of zero-divide.
        if(n_j == 0 ) return 0;
        float rate = (n_i*1.0f)/n_j;
        int idx = ceil(rate*local_idx);

        // In case of overflow
        if(idx>n_i){
            idx = n_i>1?n_i-1:0;
        }
        return idx;
    }
    
}

#ifdef MARKER
/*
    @brief For making JSK-Markers with pointclouds.
    @param cloud: The clusterred cloud to make a marker.
    @param ns: The name string.
    @param id: The marker id.
    @param r,g,b: The clour of marker.
*/
visualization_msgs::Marker mark_cluster(pcl::PointCloud<SLRPointXYZIRL>::Ptr cloud_cluster, 
    std::string ns ,int id, float r, float g, float b) { 
  Eigen::Vector4f centroid; 
  Eigen::Vector4f min; 
  Eigen::Vector4f max; 
  
  pcl::compute3DCentroid (*cloud_cluster, centroid); 
  pcl::getMinMax3D (*cloud_cluster, min, max); 
  
  uint32_t shape = visualization_msgs::Marker::CUBE; 
  visualization_msgs::Marker marker; 
  marker.header.frame_id = "/velodyne"; 
  marker.header.stamp = ros::Time::now(); 
  
  marker.ns = ns; 
  marker.id = id; 
  marker.type = shape; 
  marker.action = visualization_msgs::Marker::ADD; 
  
  marker.pose.position.x = centroid[0]; 
  marker.pose.position.y = centroid[1]; 
  marker.pose.position.z = centroid[2]; 
  marker.pose.orientation.x = 0.0; 
  marker.pose.orientation.y = 0.0; 
  marker.pose.orientation.z = 0.0; 
  marker.pose.orientation.w = 1.0; 
  
  marker.scale.x = (max[0]-min[0]); 
  marker.scale.y = (max[1]-min[1]); 
  marker.scale.z = (max[2]-min[2]); 
  
  if (marker.scale.x ==0) 
      marker.scale.x=0.1; 

  if (marker.scale.y ==0) 
    marker.scale.y=0.1; 

  if (marker.scale.z ==0) 
    marker.scale.z=0.1; 
    
  marker.color.r = r; 
  marker.color.g = g; 
  marker.color.b = b; 
  marker.color.a = 0.5; 

  marker.lifetime = ros::Duration(0.5); 
//   marker.lifetime = ros::Duration(0.5); 
  return marker; 
}
#endif


#ifdef IO
int tab=0;
#endif

/*
    @brief Velodyne pointcloud callback function, which subscribe `/all_points`
    and publish cluster points `slr`.
*/
void ScanLineRun::velodyne_callback_(const sensor_msgs::PointCloud2ConstPtr& in_cloud_msg){
    // Msg to pointcloud
    pcl::PointCloud<SLRPointXYZIRL> laserCloudIn;
    pcl::fromROSMsg(*in_cloud_msg, laserCloudIn);

    /// Clear and init.
    // Clear runs in the previous scan.
    max_label_ = 1;
    if(!runs_.empty()){
        runs_.clear();
        runs_.push_back(dummy_);// dummy for index `0`
        runs_.push_back(dummy_);// for ground points
    }
    
    // Init laser frame.
    SLRPointXYZIRL p_dummy;
    p_dummy.intensity = -1;
    laser_row_ = std::vector<SLRPointXYZIRL> (2251, p_dummy);
    laser_frame_ = std::vector<std::vector<SLRPointXYZIRL> >(32, laser_row_);
    
    // Init non-ground index holder.
    for(int i=0;i<sensor_model_;i++){
        ng_idx_[i].clear();
    }

#ifdef INTEREST_ONLY
    interest_idx_ += 1;
#endif

    // Organize Pointcloud in scanline
    double range = 0;
    int row = 0;
    for(auto &point:laserCloudIn.points){
        if(point.ring<sensor_model_&&point.ring>=0){
            
#ifdef INTEREST_ONLY
            // Set the intensity of non-interested points to zero.
            point.intensity = 0;
#endif
            // Compute and angle. 
            // @Note: In this case, `x` points right and `y` points forward.
            range = sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
            if(point.x>=0){
                row = int(563 - asin(point.y/range)/0.00279111);
            }else if(point.x<0 && point.y <=0){
                row = int(1688 + asin(point.y/range)/0.00279111);
            }else {
                row = int(1688 + asin(point.y/range)/0.00279111);
            }

            if(row>2250||row<0){
                ROS_ERROR("Row: %d is out of index.", row);
                return;
            }else{
                laser_frame_[point.ring][row] = point;
            }
            
            if(point.label != 1u){
                ng_idx_[point.ring].push_back(row);
            }else{
                runs_[1].insert_after(runs_[1].cbefore_begin(), &point);
            }
        } 
    }


    // Main processing
    for(int i=0;i<sensor_model_;i++){
        // get runs on current scan line i
        find_runs_(i);
        update_labels_(i);
    }
    
#ifdef INTEREST_ONLY
    ROS_INFO("Showing interested points only line: %d idx: %d", interest_line_, interest_idx_);
    
    for(int i=interest_line_;i>=0;i--){
        if(i==interest_line_){
            auto &point_d = laser_frame_[i][interest_idx_];
            // Empty dummy point
            if(point_d.intensity == -1)
                continue;
            point_d.intensity = i;
            // Add key point
            runs_[0].insert_after(runs_[0].cbefore_begin(), &laser_frame_[i][interest_idx_]);
        }else{
            // Add neighbour point with smart idx
            auto &point_d = laser_frame_[i][interest_idx_];
            if(point_d.intensity == -1)
                continue;
            point_d.intensity = i;
            // Add neighbour point
            runs_[0].insert_after(runs_[0].cbefore_begin(), &laser_frame_[i][interest_idx_]);
        }
    }
#endif
    // Extract Clusters
    // re-organize scan-line points into cluster point cloud
    pcl::PointCloud<SLRPointXYZIRL>::Ptr laserCloud(new pcl::PointCloud<SLRPointXYZIRL>());
    pcl::PointCloud<SLRPointXYZIRL>::Ptr clusters(new pcl::PointCloud<SLRPointXYZIRL>());

#ifdef IO
    // should be modified
    std::string str_path = "./PCD";
    if(!boost::filesystem::exists(str_path)){
        boost::filesystem::create_directories(str_path);
    }
    boost::format fmt1(str_path + "/%d_%d.%s");
    pcl::PointCloud<SLRPointXYZIRL>::Ptr to_save(new pcl::PointCloud<SLRPointXYZIRL>());
    pcl::PointCloud<SLRPointXYZIRL>::Ptr to_save_all(new pcl::PointCloud<SLRPointXYZIRL>());
#endif

#ifdef MARKER    
    visualization_msgs::MarkerArray ma;
#endif
    int cnt = 0;

    // Re-organize pointcloud clusters for PCD saving or publish
#ifdef INTEREST_ONLY
    for(size_t i=0;i<1;i++){
#else
    for(size_t i=2;i<runs_.size();i++){
#endif
        if(!runs_[i].empty()){
            cnt++;

            int ccnt = 0;
            // adding run current for publishing
            for(auto &p:runs_[i]){
                // Reorder the label id
                ccnt++;
                p->label = cnt;
                laserCloud->points.push_back(*p);
                // clusters->points.push_back(*p);
#ifdef IO
                to_save_all->points.push_back(*p);
                to_save->points.push_back(*p);
#endif
            }

#ifdef IO
            if(tab==1){
                pcl::io::savePCDFileBinary((fmt1%(tab)%(cnt)%"pcd").str(), *to_save);
                to_save->clear();
            } 
#endif   

#ifdef INTEREST_ONLY
            // Counting interested point and its neighbour points.
            ROS_INFO("cluster i:%d, size:%d",i, ccnt);
#endif
            // clusters->clear();
        }
    }
    ROS_INFO("Total cluster: %d", cnt);
    // Publish Cluster Points
    if(laserCloud->points.size()>0){
        sensor_msgs::PointCloud2 cluster_msg;
        pcl::toROSMsg(*laserCloud, cluster_msg);
        cluster_msg.header.frame_id = "/velodyne";
        cluster_points_pub_.publish(cluster_msg);
    }
#ifdef IO
    // Only save the clusterred results of the first frame.
    if(tab==1){
        to_save_all->height = to_save_all->points.size();
        to_save_all->width = 1;
        pcl::io::savePCDFileASCII((fmt1%(0)%(0)%"pcd").str(), *to_save_all);
    }
    tab++;
#endif
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "ScanLineRun");
    ScanLineRun node;
    ros::spin();

    return 0;

}