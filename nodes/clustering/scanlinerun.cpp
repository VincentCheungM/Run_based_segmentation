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

#include <iostream>
// For disable PCL complile lib, to use PointXYZIR, and custome pointcloud    
#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <velodyne_pointcloud/point_types.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/impl/octree_search.hpp>
#include <set>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

#include <pcl/io/pcd_io.h>
#include <boost/format.hpp>
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

/*
    @brief Scan Line Run ROS Node.
    @param Velodyne Pointcloud Non Ground topic.
    @param Sensor Model.
    @param Threshold between points belong to the same run
    @param Threshold between runs
    
    @subscirbe:/points_no_ground
    @publish:/cluster
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

    int sensor_model_;// also means number of sensor scan line
    double th_run_;
    double th_merge_;

    std::vector<pcl::PointCloud<SLRPointXYZIRL>::Ptr > laserCloudScans_;// holding all lines
    std::vector<RUN::Ptr> runs_;// holding all runs
    std::vector<std::vector<int> > line_run_idx_;// holding run labels on each line
    int label_;// run labels

    void velodyne_callback_(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);
    void find_runs_(int ring);
    void update_labels_(int ring);
    // void extract_clusters_(void);
    // Model parameter for ground plane fitting
    ros::Publisher marker_array_pub_;
};    

/*
    @brief Constructor of SLR Node.
    @return void
*/
ScanLineRun::ScanLineRun():node_handle_("~"){
    // Init ROS related
    ROS_INFO("Inititalizing Scan Line Run Cluster...");
    node_handle_.param<std::string>("point_topic", point_topic_, "/points_no_ground");
    ROS_INFO("point_topic: %s", point_topic_.c_str());

    node_handle_.param("sensor_model", sensor_model_, 32);
    ROS_INFO("Sensor Model: %d", sensor_model_);
    // Init Ptrs
    for(int i=0;i<sensor_model_;i++){
        // sensor_model_ lines
        pcl::PointCloud<SLRPointXYZIRL>::Ptr item( new pcl::PointCloud<SLRPointXYZIRL>());
        laserCloudScans_.push_back(item);
        std::vector<int> idx_item;
        line_run_idx_.push_back(idx_item);
    }

    node_handle_.param("th_run", th_run_, 0.2);
    ROS_INFO("Point-to-Run Threshold: %f", th_run_);

    node_handle_.param("th_merge", th_merge_, 0.8);
    ROS_INFO("RUN-to-RUN Distance Threshold: %f", th_merge_);

    // Listen to velodyne topic
    points_node_sub_ = node_handle_.subscribe(point_topic_, 2, &ScanLineRun::velodyne_callback_, this);
    
    // Publish Init
    std::string cluster_topic, ring_topic;
    node_handle_.param<std::string>("cluster", cluster_topic, "/slr");
    ROS_INFO("Cluster Output Point Cloud: %s", cluster_topic.c_str());
    // node_handle_.param<std::string>("ring_point_topic_for_debug", ring_topic, "/ring");
    // ROS_INFO("ring_point_topic_for_debug Output Point Cloud: %s", ring_topic.c_str());
    
    cluster_points_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2 >(cluster_topic, 10);
    // ground_points_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>(ring_topic, 2);
    marker_array_pub_ = node_handle_.advertise<visualization_msgs::MarkerArray>("cluster_ma", 10);
}


/*
    @brief Read Points from the given scan_line. Using Octree to construct a 
    tree, and perform Radius Search on points. Points within radius will be labelled
    to label of the lowest point. Points stored in `laserCloudScans_[scan_line]`. 
    
    @param scan_line: the scan line to find runs.
*/
void ScanLineRun::find_runs_(int scan_line){
    float resolution = 32.0f;
    pcl::octree::OctreePointCloudSearch<SLRPointXYZIRL> octree (resolution);
    octree.setInputCloud(laserCloudScans_[scan_line]);
    octree.addPointsFromInputCloud();   
    // Neighbors within radius search
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    SLRPointXYZIRL searchPoint;
    SLRPointXYZIRL first_point;
    SLRPointXYZIRL last_point;
    uint16_t target_label = 0;

    // This will mark or nearest neighbour of search point from Octree Radius Search.
    // Radius search is perform on every points, and then points will be labeled and cluster into runs_.

    for(size_t idx =0; idx<laserCloudScans_[scan_line]->points.size();idx++){
        // iterate each point to search
        searchPoint = laserCloudScans_[scan_line]->points[idx];
        if(searchPoint.label != 0){
            // this point has been iterated, skip this point           
            continue;
        }

        
        // if search, marked the points into a run
        // Here mared all nearest neighbour points into a run
        if(octree.radiusSearch(searchPoint, th_run_, pointIdxRadiusSearch, pointRadiusSquaredDistance)){
            sort(pointIdxRadiusSearch.begin(),pointIdxRadiusSearch.end());
            // if the first point and the last point are not cluster, label it to 1 as the first cluster
            first_point = laserCloudScans_[scan_line]->points[pointIdxRadiusSearch[0]];
            last_point = laserCloudScans_[scan_line]->points[pointIdxRadiusSearch[pointIdxRadiusSearch.size()-1]];
            
            if(first_point.label == 0 && last_point.label == 0){
                // Adding new run, none of points has been labeled.
                RUN::Ptr item(new RUN());
                runs_.push_back(item);
                target_label = ++label_;
                line_run_idx_[scan_line].push_back(target_label);
            }else if(last_point.label == 0){
                // the early points has been visited
                target_label = first_point.label;
            }else if(first_point.label == 0){
                // the later points has been visited, happend at the end of circle
                target_label = last_point.label;
            }else{
                // otherwise, take the small one as label. For smallest label comes from the bottom scan_line
                target_label = min(first_point.label, last_point.label);
            }
            // set point label
            for (size_t i = 1; i < pointIdxRadiusSearch.size(); ++i){
                // try not to make data dirty
                // set new label and adding to run
                if(laserCloudScans_[scan_line]->points[pointIdxRadiusSearch[i]].label != target_label){
                    laserCloudScans_[scan_line]->points[pointIdxRadiusSearch[i]].label = target_label;
                    runs_[target_label-1]->push_back(laserCloudScans_[scan_line]->points[pointIdxRadiusSearch[i]]);
                }
            } 
        }

    }

}

/*
    @brief Update label between two runs.
*/
void ScanLineRun::update_labels_(int scan_line){
    // runs above 
    pcl::PointCloud<SLRPointXYZIRL>::Ptr run_above(new pcl::PointCloud<SLRPointXYZIRL>());
    for(auto run_id_above_line:line_run_idx_[scan_line-1]){
        // cout<<"i:"<<i<<" ";
        *run_above += *(runs_[run_id_above_line-1]);
    }
    // Octree search
    float resolution = 32.0f;
    pcl::octree::OctreePointCloudSearch<SLRPointXYZIRL> octree(resolution);
    octree.setInputCloud(run_above);
    octree.addPointsFromInputCloud();
    // Neighbors within radius search
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;   
    // Temp var to be used later. Need init for every iterate.
    SLRPointXYZIRL searchPoint;
    RUN::Ptr run_current;
    std::set<uint16_t> label_to_merge;
    uint16_t target_label = 0;
    // Used for merge labels. 
    std::set<int> new_label;

    for(auto run_in_line:line_run_idx_[scan_line]){
        run_current = runs_[run_in_line-1];
        target_label = 0;
        // Clear label_to_merge
        label_to_merge.clear();

        for(int p_idx=0;p_idx<run_current->points.size();p_idx++){
            searchPoint = run_current->points[p_idx];
            // search the nearest point to merge
            // get the nearest point within th_merge_ to merge
            if(octree.radiusSearch(searchPoint, th_merge_, pointIdxRadiusSearch, pointRadiusSquaredDistance, 1)>0){
                label_to_merge.insert(run_above->points[pointIdxRadiusSearch[0]].label);
                if(target_label ==0){
                    target_label = run_above->points[pointIdxRadiusSearch[0]].label;
                }else{
                    target_label = min(target_label, run_above->points[pointIdxRadiusSearch[0]].label);
                }
            }
        }

        // If label_to_merge is not empty, then update label
        if(!label_to_merge.empty()){
            // set current point to target label
            for(int p_idx=0;p_idx<run_current->points.size();p_idx++){
                run_current->points[p_idx].label = target_label;
            }

            // connect current run into the above target run
            *runs_[target_label-1]+= *(run_current);
            // remove current run
            run_current->clear();
            // Merge Other Run in the label_to_merge set, mentioned as `MergedLabel`.
            // The conflict is solved according the the `two-run segmentation` paper.
            // All run except for the target run, will be merged into target run.
            RUN::Ptr run_to_merge;
            for(auto label_id:label_to_merge){
                // if the label_to_merge is not target_label,
                // merge that run into target run.
                if(label_id!=target_label && runs_[label_id-1]->points.size()>0){
                    run_to_merge = runs_[label_id-1];
                    // Update labels
                    for(int p_idx=0;p_idx<run_to_merge->points.size();p_idx++){
                        run_to_merge->points[p_idx].label = target_label;
                    }
                    // Merge into target run
                    *runs_[target_label-1]+= *(run_to_merge);
                    run_to_merge->clear();
                }
            }
            // update label on the current run line
            new_label.insert(target_label);
            
        }else{
            // keep current run inside the line_run_idx
            new_label.insert(run_in_line);
            // Do nothing
            // In this implmentation, the current Node has its own label after find_runs.
        }


    }
    // Update the line_run_idx with merged label, so that it can be used in the next scan_line
    // Here set it's used, to avoid multi-add label
    std::vector<int> merged_label;
    for(auto s:new_label){
        merged_label.push_back(s);
    }
    // New run labels of current scan_line
    line_run_idx_[scan_line] = merged_label;

}


visualization_msgs::Marker mark_cluster(pcl::PointCloud<SLRPointXYZIRL>::Ptr cloud_cluster, std::string ns ,int id, float r, float g, float b) 
{ 
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

#ifdef IO
int tab=0;
#endif

/*
    @brief Velodyne pointcloud callback function. The main GPF pipeline is here.
    PointCloud SensorMsg -> Pointcloud -> z-value sorted Pointcloud
    ->error points removal -> extract ground seeds -> ground plane fit mainloop
*/
void ScanLineRun::velodyne_callback_(const sensor_msgs::PointCloud2ConstPtr& in_cloud_msg){
    // Msg to pointcloud
    pcl::PointCloud<VPoint> laserCloudIn;
    pcl::fromROSMsg(*in_cloud_msg, laserCloudIn);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);

    // Clear points in the previous scan
    runs_.clear();
    label_ = 0;
    for(int i=0;i<sensor_model_;i++){
        laserCloudScans_[i]->clear();
        line_run_idx_[i].clear();
    }
    
    // Organize Pointcloud in scanline
    SLRPointXYZIRL point;
    for(int i=0;i<laserCloudIn.points.size();i++){
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;
        point.intensity = laserCloudIn.points[i].intensity;
        point.ring = laserCloudIn.points[i].ring;
        point.label = 0u;// 0 means uncluster

        if(point.ring<=sensor_model_&&point.ring>=0){
            laserCloudScans_[point.ring]->push_back(point);
        }    
    }
    
    // Mainloop
    // find run of the first scan line
    find_runs_(0);
    for(int i=1;i<sensor_model_;i++){
        // get runs on current scan line i
        find_runs_(i);
        laserCloudScans_[i]->clear();
        if(line_run_idx_[i-1].size()>0&&line_run_idx_[i].size()>0)
            update_labels_(i);
        // update labels between two runs
        // iterate new run
    }
    
    // Extract Clusters
    // re-organize scan-line points into cluster point cloud
    pcl::PointCloud<SLRPointXYZIRL>::Ptr laserCloud(new pcl::PointCloud<SLRPointXYZIRL>());

#ifdef IO
    // should be modified
    boost::format fmt1("/media/es/beta/velodyne/Origin-PCD/%d_%d.%s");
#endif
    // int id = 0;
    // int id_s = 0;
    // int not_zeros = 0;
    // visualization_msgs::MarkerArray ma;
    // Re-organize pointcloud clusters for PCD saving or publish
    for(int i=0;i<runs_.size();i++){
        // if(runs_[i]->points.size()>id_s){
        //     id = i;
        //     id_s = runs_[i]->points.size();
        // }
        if(runs_[i]->points.size()!=0){
            // cout<<"run id: "<<i<<" size: "<<runs_[i]->points.size()<<" not zero "<<runs_.size()<<endl;
            // not_zeros++;
#ifdef IO
            pcl::io::savePCDFileBinary((fmt1%(tab)%(i)%"pcd").str(), *runs_[i]); 
#endif      
            // adding run current for publishing
            *laserCloud+=*runs_[i];
            //visualization_msgs::Marker mac = mark_cluster(runs_[i],std::to_string(i),i,i/10,i/5,i*2);
            //ma.markers.push_back(mac);
        }
    }

    // Publish Cluster Points
    if(laserCloud->points.size()>0){
        //*laserCloud+=*runs_[id];
        //cout<<"largest id: "<<id<<" size: "<<id_s<<" runs_.size(): "<<not_zeros<<endl;
        sensor_msgs::PointCloud2 cluster_msg;
        pcl::toROSMsg(*laserCloud, cluster_msg);
        cluster_msg.header.frame_id = "/velodyne";
        cluster_points_pub_.publish(cluster_msg);
    }
#ifdef IO
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