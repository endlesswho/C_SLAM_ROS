#include <iostream>
#include <fstream>
#include <math.h>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/Imu.h"
#include "gps_common/GPSFix.h"

using namespace std;
using namespace ros;

double timeOdometry = 0;
double throttle = 1;
bool last_frame_exist = false;
bool cloudCBRes = false;

int idx = 0;
nav_msgs::Odometry gps_Odometry;
nav_msgs::Odometry loam_Odometry;
double latitude_1;
double latitude_2;
double longitude_1;
double longitude_2;
double altitude_2;
bool init_flag_1 = true;
bool init_flag_2 = true;
visualization_msgs::Marker marker;

// create markers
visualization_msgs::Marker makeMarkers(int id, double x, double y, double z){
    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = id;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    return marker;
}

void getPoseHandler(const nav_msgs::Odometry::ConstPtr& odomAftMapped){
/*
    if(init_flag_1){
        longitude_1 = odomAftMapped->pose.pose.position.z;
        latitude_1 = odomAftMapped->pose.pose.position.x;
        init_flag_1 = false;
    }
*/
    loam_Odometry.pose.pose.position.x = -odomAftMapped->pose.pose.position.z;
    loam_Odometry.pose.pose.position.y = -odomAftMapped->pose.pose.position.x;
    loam_Odometry.pose.pose.position.z = odomAftMapped->pose.pose.position.y;
    loam_Odometry.pose.pose.orientation = odomAftMapped->pose.pose.orientation;
}

//void getGPSHandler(const sensor_msgs::NavSatFix::ConstPtr& gps) {
//    if(init_flag_2){
//        longitude_2 = gps->longitude;
//        latitude_2 = gps->latitude;
//        init_flag_2 = false;
//    }
//
//    double delta_longitude = (gps->longitude - longitude_2)*M_PI/180.0;
//    double delta_latitude = (gps->latitude - latitude_2)*M_PI/180.0;
//    double media_latitude = (gps->latitude + latitude_2)*M_PI/360.0;
//    double delta_x = delta_longitude*6367000*cos(media_latitude);
//    double delta_y = 6367000*delta_latitude;
//
//    gps_Odometry.pose.pose.position.x = delta_x;
//    gps_Odometry.pose.pose.position.y = delta_y;
//    gps_Odometry.pose.pose.position.z = gps->altitude;
//    marker = makeMarkers(idx, gps_Odometry.pose.pose.position.x, gps_Odometry.pose.pose.position.y,
//                         gps_Odometry.pose.pose.position.z);
//    idx ++;
//    cloudCBRes = true;
//}

void getGPSHandler(const gps_common::GPSFix::ConstPtr& gps) {
    if(init_flag_2){
        longitude_2 = gps->longitude;
        latitude_2 = gps->latitude;
        altitude_2 = gps->altitude;
        init_flag_2 = false;
    }

    double delta_longitude = (gps->longitude - longitude_2)*M_PI/180.0;
    double delta_latitude = (gps->latitude - latitude_2)*M_PI/180.0;
    double media_latitude = (gps->latitude + latitude_2)*M_PI/360.0;
    double delta_x = delta_longitude*6367000*cos(media_latitude);
    double delta_y = 6367000*delta_latitude;

    gps_Odometry.pose.pose.position.x = delta_x;
    gps_Odometry.pose.pose.position.y = delta_y;
    gps_Odometry.pose.pose.position.z = gps->altitude-altitude_2;
    marker = makeMarkers(idx, gps_Odometry.pose.pose.position.x, gps_Odometry.pose.pose.position.y,
                         gps_Odometry.pose.pose.position.z);
    idx ++;
    cloudCBRes = true;
}

//void getQuatHandler(const geometry_msgs::QuaternionStamped::ConstPtr& quat) {
//    geometry_msgs::Quaternion Quat = quat->quaternion;
//    gps_Odometry.pose.pose.orientation.x = Quat.x;
//    gps_Odometry.pose.pose.orientation.y = Quat.y;
//    gps_Odometry.pose.pose.orientation.z = Quat.z;
//    gps_Odometry.pose.pose.orientation.w = Quat.w;
//    cloudCBRes = true;
//}

void getQuatHandler(const sensor_msgs::Imu::ConstPtr& quat) {
    geometry_msgs::Quaternion Quat = quat->orientation;
    gps_Odometry.pose.pose.orientation.x = Quat.x;
    gps_Odometry.pose.pose.orientation.y = Quat.y;
    gps_Odometry.pose.pose.orientation.z = Quat.z;
    gps_Odometry.pose.pose.orientation.w = Quat.w;
    cloudCBRes = true;
}

// TODO evaluation the Translation Error
void evaluation(){

}

int main (int argc, char **argv)
{
    ros::init (argc, argv, "pcl_write");

    ros::NodeHandle nh;
//    ros::Subscriber subGPS = nh.subscribe<sensor_msgs::NavSatFix>
//            ("/gps", 5, getGPSHandler);
    ros::Subscriber subGPS = nh.subscribe<gps_common::GPSFix>
            ("/gps", 5, getGPSHandler);
//    ros::Subscriber subQuat = nh.subscribe<geometry_msgs::QuaternionStamped>
//            ("/imu", 5, getQuatHandler);
    ros::Subscriber subQuat = nh.subscribe<sensor_msgs::Imu>
            ("/imu", 5, getQuatHandler);
    // subscribe a topic
    ros::Subscriber subOdomAftIntegrated = nh.subscribe<nav_msgs::Odometry>
            ("/integrated_to_init", 5, getPoseHandler);

    ros::Publisher pubGPSOdometry = nh.advertise<nav_msgs::Odometry>
            ("/gps_Odometry", 5);
    ros::Publisher pubLoamOdometry = nh.advertise<nav_msgs::Odometry>
            ("/loam_Odometry", 5);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>
            ("visualization_marker", 10);

    ofstream fout("./gym3_gps.txt");
    ofstream fout2("./gym3_loam.txt");
    ros::Rate rate(200);
    bool status = ros::ok();
    while(status){
        ros::spinOnce();
        if(cloudCBRes)
        {
            gps_Odometry.header.frame_id = "map";
            marker_pub.publish(marker);
            pubGPSOdometry.publish(gps_Odometry);
            loam_Odometry.header.frame_id = "map";
            pubLoamOdometry.publish(loam_Odometry);
            fout<<gps_Odometry.pose.pose.position.x<<" "<<gps_Odometry.pose.pose.position.y<<" "<<gps_Odometry.pose.pose.position.z<<endl;
            fout2<<loam_Odometry.pose.pose.position.x<<" "<<loam_Odometry.pose.pose.position.y<<" "<<loam_Odometry.pose.pose.position.z<<endl;
        }
        status = ros::ok();
        rate.sleep();
    }
    fout.close();
    return 0;
}


