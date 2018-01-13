#include <iostream>
#include <map>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <tf/transform_listener.h>

#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/io.h>

#include <pcl/common/time.h>
#include <pcl/console/parse.h>

#include "../tracking/TrackManager.h"

#include <qapplication.h>

#include <string>

#include "ros_bridge/cloud_odom_ros_subscriber.h"

#include "../depth_clustering/clusterers/image_based_clusterer.h"
#include "../depth_clustering/ground_removal/depth_ground_remover.h"
#include "../depth_clustering/projections/ring_projection.h"
#include "../depth_clustering/projections/spherical_projection.h"
#include "../depth_clustering/utils/radians.h"
#include "../depth_clustering/visualization/cloud_saver.h"
#include "../depth_clustering/visualization/visualizer.h"

#include "../depth_clustering/tclap/CmdLine.h"


using namespace std;
using namespace depth_clustering;

pcl::PointCloud<pcl::PointXYZ>::Ptr last_cloudPointer(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloudPointer(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles_pcl(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr total_cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_segmented_result (new pcl::PointCloud<pcl::PointXYZRGB>());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>());
std::vector<int> indices;
std::vector<int> indexs;
std::vector< Matrix<float,6,1> > last_objs;
std::vector< Matrix<float,6,1> > current_objs;
// time cost too much
visualization_msgs::MarkerArray marker_list;
visualization_msgs::MarkerArray velocity_list;
ofstream write_file;
double timeOdometry = 0;
int obj_idx = 0;
int obj_index = 0;
bool cloudCBRes = false;
double duration;
const float DISTANCE_XY = 100;

TrackManager trackManager;

// get Bound of the cluster
void getBound(pcl::PointXYZL min_point, pcl::PointXYZL max_point, Eigen::Vector4f &size)
{
    size(0)=max_point.x-min_point.x;
    size(1)=max_point.y-min_point.y;
    size(2)=max_point.z-min_point.z;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr getColoredCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, std::vector <pcl::PointIndices> clusters_indices)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;

    if (!clusters_indices.empty ())
    {
        colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared ();

        srand (static_cast<unsigned int> (time (0)));
        std::vector<unsigned char> colors;
        for (size_t i_segment = 0; i_segment < clusters_indices.size (); i_segment++)
        {
            colors.push_back (static_cast<unsigned char> (rand () % 256));
            colors.push_back (static_cast<unsigned char> (rand () % 256));
            colors.push_back (static_cast<unsigned char> (rand () % 256));
        }

        colored_cloud->width = cloud_in->width;
        colored_cloud->height = cloud_in->height;
        colored_cloud->is_dense = cloud_in->is_dense;
        for (size_t i_point = 0; i_point < cloud_in->points.size (); i_point++)
        {
            pcl::PointXYZRGB point;
            point.x = *(cloud_in->points[i_point].data);
            point.y = *(cloud_in->points[i_point].data + 1);
            point.z = *(cloud_in->points[i_point].data + 2);
            point.r = 255;
            point.g = 0;
            point.b = 0;
            colored_cloud->points.push_back (point);
        }

        std::vector< pcl::PointIndices >::iterator i_segment;
        int next_color = 0;
        for (i_segment = clusters_indices.begin (); i_segment != clusters_indices.end (); i_segment++)
        {
            std::vector<int>::iterator i_point;
            for (i_point = i_segment->indices.begin (); i_point != i_segment->indices.end (); i_point++)
            {
                int index;
                index = *i_point;
                colored_cloud->points[index].r = colors[3 * next_color];
                colored_cloud->points[index].g = colors[3 * next_color + 1];
                colored_cloud->points[index].b = colors[3 * next_color + 2];
            }
            next_color++;
        }
    }

    return (colored_cloud);
}

// create markers
visualization_msgs::Marker makeMarkers(int obj_index, Eigen::Matrix<float, 7, 1> obj){
    int id = obj(6);
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/velodyne";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = obj_index;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = obj(0);
    marker.pose.position.y = obj(1);
    marker.pose.position.z = obj(2);
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.1;//obj(3);
    marker.scale.y = 0.1;//obj(4);
    marker.scale.z = 0.1;//obj(5);

    switch ( id ) {
        case 0:
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            // Code
            break;
        case 1:
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            // Code
            break;
        case 2:
            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 1.0f;
            break;
        default:
            // Code
            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            break;
    }

    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    return marker;
}

// create velocity markers
visualization_msgs::Marker makeVelocityMarkers(int id,Eigen::Vector4f point_start, Eigen::Vector4f point_end){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/velodyne";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 0.1;

    // Line list is green
    marker.color.g = 0.2*id;
    marker.color.a = 1.0;

    geometry_msgs::Point p_1, p_2;
    p_1.x = point_start(0);
    p_1.y = point_start(1);
    p_1.z = point_start(2);

    p_2.x = point_end(0);
    p_2.y = point_end(1);
    p_2.z = point_end(2);

    marker.points.push_back(p_1);
    marker.points.push_back(p_2);

    marker.lifetime = ros::Duration();
    return marker;
}

Cloud::Ptr cloud_cast(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in) {
    Cloud::Ptr cloud(new Cloud);
    RichPoint point;
    for (size_t i_point = 0; i_point < cloud_in->points.size (); i_point++)
    {
        point.x() = *(cloud_in->points[i_point].data);
        point.y() = *(cloud_in->points[i_point].data + 1);
        point.z() = *(cloud_in->points[i_point].data + 2);
        cloud->push_back(point);
    }
    return cloud;
}

void segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in)
{
    pcl::PointXYZL min_point, max_point;
    Eigen::Vector4f size;
    Eigen::Matrix<float,6,1> obj;

    vector<Cloud>currentframe_cluster;
    std::vector <pcl::PointCloud<pcl::PointXYZRGB> > clusters_rgb;

    int min_cluster_size = 100;
    int max_cluster_size = 200000;

    int smooth_window_size = 9;
    Radians ground_remove_angle = 7_deg;

    Radians angle_tollerance = Radians::FromDegrees(10);
    auto proj_params_ptr = ProjectionParams::VLP_16();

    // step 1 Remove Ground
    auto depth_ground_remover = DepthGroundRemover(
            *proj_params_ptr, ground_remove_angle, smooth_window_size);

    // step 2 image labeler cluster
    ImageBasedClusterer<LinearImageLabeler<>> clusterer(
            angle_tollerance, min_cluster_size, max_cluster_size);

    clusterer.SetDiffType(DiffFactory::DiffType::ANGLES);

    depth_ground_remover.AddClient(&clusterer);

    auto cloud = cloud_cast(cloud_in);

    cloud->InitProjection(*proj_params_ptr);

    depth_ground_remover.OnNewObjectReceived(*cloud, 0);

    currentframe_cluster = clusterer.getClusters();

    for(size_t i=0;i<currentframe_cluster.size();i++){
        auto tmp = currentframe_cluster[i].ToPcl();
        getMinMax3D(*tmp,min_point,max_point);
        getBound(min_point,max_point,size);
        // get cluster center
        obj(0) = (max_point.x+min_point.x)/2;
        obj(1) = (max_point.y+min_point.y)/2;
        obj(2) = (max_point.z+min_point.z)/2;
        obj(3) = size(0);
        obj(4) = size(1);
        obj(5) = size(2);
        if(size(0)<2 && size(1)<2 && size(2)<2 && size(2)>0.2)
        {
            current_objs.push_back(obj);
        }
    }

    vector<Eigen::Matrix<float, 7, 1> > current_objs_idx = trackManager.update(current_objs);
    // store markers
#if 1
    for(size_t i=0; i<current_objs_idx.size();i++){
        Eigen::Matrix<float, 7, 1> obj;
        obj = current_objs_idx[i];
        marker_list.markers.push_back(makeMarkers(obj_index, obj));
        obj_index++;
    }
#endif
    current_objs.clear();
}

void cloudCB(const sensor_msgs::PointCloud2 &input)
{
    indexs.clear();
    cloud_filtered.reset();
    pcl::fromROSMsg(input, *current_cloudPointer);

    double distance = 0;
    for(size_t i=0; i<current_cloudPointer->size(); i++) {
        distance = pow((current_cloudPointer->points[i].x), 2) + pow((current_cloudPointer->points[i].y), 2);
        if(distance<=DISTANCE_XY && (current_cloudPointer->points[i].z)>-0.6)
            indexs.push_back(i);
    }		//select the point whose distance less than 400

    pcl::copyPointCloud(*current_cloudPointer, indexs, *obstacles_pcl);

    //segmentation
    segmentation(obstacles_pcl);
    //cloud_segmented_result = cluster_filter(obstacles_pcl);

    cloudCBRes = true;
}

int main (int argc, char **argv)
{
    ros::init (argc, argv, "pcl_write");

    ros::NodeHandle nh;
    ros::Subscriber velodyne_cloud_sub = nh.subscribe("velodyne_points", 1, cloudCB);
    ros::Publisher pubObstacles = nh.advertise<sensor_msgs::PointCloud2>
                                           ("/obstacles", 1);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 10);
    // ros::Publisher velocity_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_velocity", 10);

    ros::Rate rate(1000);

    // write_file.open("outfile.txt");
    bool status = ros::ok();
    while(status){
    	ros::spinOnce();

    	if(cloudCBRes) {
            sensor_msgs::PointCloud2 obstacles_pub;
            pcl::toROSMsg(*cloud_segmented_result, obstacles_pub);
            obstacles_pub.header.stamp = ros::Time().fromSec(timeOdometry);
            obstacles_pub.header.frame_id = "/velodyne";
            pubObstacles.publish(obstacles_pub);

            // publish marker list
            marker_pub.publish(marker_list);
            // publish velocity list
            // velocity_pub.publish(velocity_list);

            marker_list.markers.clear();
            //velocity_list.markers.clear();

        }
    	status = ros::ok();
    	rate.sleep();
    }
    return 0;
}
