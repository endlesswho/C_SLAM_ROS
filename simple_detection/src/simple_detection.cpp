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
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/io.h>

#include <pcl/common/time.h>
#include <pcl/console/parse.h>

#include "../tracking/TrackManager.h"


using namespace std;

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
const float DISTANCE_XY = 64;

TrackManager trackManager;

// get Bound of the cluster
void getBound(pcl::PointXYZRGB min_point, pcl::PointXYZRGB max_point, Eigen::Vector4f &size)
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

void calc_NN()
{
    if(last_objs.empty())
    {
        last_objs=current_objs;
        return;
    }
    // calc NN and V
    pcl::PointCloud<pcl::PointXYZ>::Ptr last_center(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_center(new pcl::PointCloud<pcl::PointXYZ>);
    cout<<"The size of center_last: "<<last_objs.size()<<endl;
    for(int i=0;i<last_objs.size();i++)
    {
        pcl::PointXYZ tmp;
        tmp.x=last_objs[i](0);
        tmp.y=last_objs[i](1);
        tmp.z=last_objs[i](2);
        last_center->push_back(tmp);
    }
    for(int i=0;i<current_objs.size();i++)
    {
        pcl::PointXYZ tmp;
        tmp.x=current_objs[i](0);
        tmp.y=current_objs[i](1);
        tmp.z=current_objs[i](2);

        current_center->push_back(tmp);

    }

    // make a compare between last frame and current frame and get the inliners and outliners

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(last_center);
    // cout<<current_cloudPointer->size()<<"      "<<last_cloudPointer->size()<<endl;
    pcl::getApproximateIndices<pcl::PointXYZ>(current_center, last_center, indices);		//match
    // cout<<current_cloud.size()<<"  "<<indices.size()<<endl;

    // disp V
    double distance = 0, Velocity=0;
    for(int i=0; i<current_center->size(); i++) {
        distance = sqrt(pow((current_center->points[i].x - last_center->points[indices[i]].x), 2) +
                   pow((current_center->points[i].y - last_center->points[indices[i]].y), 2)
                   + pow((current_center->points[i].z - last_center->points[indices[i]].z), 2));
        // get velocity small than a throttle
        if (distance<0.5){
            Eigen::Vector4f point_start, point_end;
            point_start(0) = current_center->points[i].x;
            point_start(1) = current_center->points[i].y;
            point_start(2) = current_center->points[i].z;
            point_end(0) = last_center->points[indices[i]].x;
            point_end(1) = last_center->points[indices[i]].y;
            point_end(2) = last_center->points[indices[i]].z;

            // store markers
#if 1
            velocity_list.markers.push_back(makeVelocityMarkers(i ,point_start, point_end));
            Eigen::Matrix<float, 7, 1> obj;
            obj << current_objs[i];
            marker_list.markers.push_back(makeMarkers(obj_index,obj));
            obj_index++;
#endif
            Velocity = sqrt(distance)*10;
            std::cout<<"Velocity"<<Velocity<<endl;
        }
    }

    last_objs = current_objs;
    current_objs.clear();
}

// get clusters
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in)
{
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_segmented (new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered_tmp;
    std::vector <pcl::PointIndices> clusters_indices;
    std::vector <pcl::PointCloud<pcl::PointXYZ> > clusters;
    std::vector <pcl::PointCloud<pcl::PointXYZRGB> > clusters_rgb;
//    // Estimate the normals
//   pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//   ne.setInputCloud (cloud_in);
//   pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_n (new pcl::search::KdTree<pcl::PointXYZ>());
//   ne.setSearchMethod (tree_n);
//   ne.setRadiusSearch (0.5);
//
//   ne.compute (*cloud_normals);
//    start = clock();
//   pcl::console::print_highlight ("Normals are computed and size is %lu\n", cloud_normals->points.size ());
//
//   // Region growing
//   pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> rg;
//   rg.setSmoothModeFlag (false); // Depends on the cloud being processed
//   rg.setInputCloud (cloud_in);
//   rg.setInputNormals (cloud_normals);
//
//   pcl::StopWatch watch;
//   rg.extract (clusters_indices);
//   pcl::console::print_highlight ("Extraction time: %f\n", watch.getTimeSeconds());
//   cloud_segmented = rg.getColoredCloud ();  //the point segmented will be colored

   // Euclidean Extraction
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_in);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.15); // 15cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_in);
    ec.extract (clusters_indices);

    cloud_segmented = getColoredCloud(cloud_in, clusters_indices);

   // Writing the resulting cloud into a pcd file
//   pcl::console::print_highlight ("Number of segments done is %lu\n", clusters_indices.size ());
   //   pcl::PCDWriter writer;
   //   writer.write<pcl::PointXYZRGB> ("segment_result.pcd", *cloud_segmented, false);

    pcl::PointXYZRGB min_point, max_point;

    Eigen::Vector4f size;
    Eigen::Matrix<float,6,1> obj;

    //seperate each cluster
    for (std::vector<pcl::PointIndices>::const_iterator it = clusters_indices.begin (); it != clusters_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGB> tmp;
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            tmp.push_back(cloud_segmented->points[*pit]);
        if(tmp.size()>100)
        {
            clusters_rgb.push_back(tmp);
        }
    }

    for(int i=0; i<clusters_rgb.size(); i++){
        getMinMax3D(clusters_rgb[i],min_point,max_point);
        getBound(min_point,max_point,size);
        // get cluster center
        obj(0) = (max_point.x+min_point.x)/2;
        obj(1) = (max_point.y+min_point.y)/2;
        obj(2) = (max_point.z+min_point.z)/2;
        obj(3) = size(0);
        obj(4) = size(1);
        obj(5) = size(2);
//        write_file<<"center: "<<endl;

        if(size(0)<1 && size(1)<1 && size(2)<2 && size(2)>0.2)
        {
            current_objs.push_back(obj);
            cloud_filtered_tmp+=clusters_rgb[i];
        }
    }

    cloud_filtered = cloud_filtered_tmp.makeShared();

    vector<Eigen::Matrix<float, 7, 1> > current_objs_idx = trackManager.update(current_objs);
    // store markers
#if 1
    for(int i=0; i<current_objs_idx.size();i++){
        Eigen::Matrix<float, 7, 1> obj;
        obj = current_objs_idx[i];
        marker_list.markers.push_back(makeMarkers(obj_index, obj));
        obj_index++;
    }
#endif

    current_objs.clear();
    //calc_NN();

    return cloud_filtered;
}

void cloudCB(const sensor_msgs::PointCloud2 &input)
{
    indexs.clear();
    cloud_filtered.reset();
    pcl::fromROSMsg(input, *current_cloudPointer);

    double distance = 0;
    for(int i=0; i<current_cloudPointer->size(); i++) {
        distance = pow((current_cloudPointer->points[i].x), 2) + pow((current_cloudPointer->points[i].y), 2);
        if(distance<=DISTANCE_XY && (current_cloudPointer->points[i].z)>-0.6)
            indexs.push_back(i);
    }		//select the point whose distance less than 400

    pcl::copyPointCloud(*current_cloudPointer, indexs, *obstacles_pcl);

    //segmentation
    cloud_segmented_result = cluster_filter(obstacles_pcl);

    cloudCBRes = true;
}

int main (int argc, char **argv)
{
    ros::init (argc, argv, "pcl_write");

    ros::NodeHandle nh;
    ros::Subscriber velodyne_cloud_sub = nh.subscribe("velodyne_points", 1, cloudCB);
    //s ros::Subscriber velodyne_cloud_sub = nh.subscribe("velodyne_cloud_registered", 1, cloudCB);
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


