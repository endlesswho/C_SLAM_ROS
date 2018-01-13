//
// Created by huyh on 18-1-6.
//

#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>

using namespace std;
using namespace ros;

bool callback_flag = false;
geometry_msgs::TransformStamped odom_trans;
sensor_msgs::PointCloud2 laserCloudFullRes3;
ros::Time current_time;

void OdomHandler(const nav_msgs::OdometryConstPtr& OdometryInfo){
    odom_trans.header.frame_id = "/camera_init";
    odom_trans.child_frame_id = "/world";
    odom_trans.header.stamp = OdometryInfo->header.stamp;
    odom_trans.transform.translation.x = OdometryInfo->pose.pose.position.x;
    odom_trans.transform.translation.y = OdometryInfo->pose.pose.position.y;
    odom_trans.transform.translation.z = OdometryInfo->pose.pose.position.z;
    odom_trans.transform.rotation =  OdometryInfo->pose.pose.orientation;
    static tf::TransformBroadcaster odom_broadcaster;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin( tf::Vector3(OdometryInfo->pose.pose.position.x, OdometryInfo->pose.pose.position.y, OdometryInfo->pose.pose.position.z) );
    q.setW(OdometryInfo->pose.pose.orientation.w);
    q.setX(OdometryInfo->pose.pose.orientation.x);
    q.setY(OdometryInfo->pose.pose.orientation.y);
    q.setZ(OdometryInfo->pose.pose.orientation.z);
    transform.setRotation(q);
    odom_broadcaster.sendTransform(tf::StampedTransform(transform,OdometryInfo->header.stamp,"/camera_init","/world"));
    callback_flag = true;
};

void PointCloudHandler(const sensor_msgs::PointCloud2ConstPtr& velodyne_cloud_registered){
    laserCloudFullRes3=*velodyne_cloud_registered;
    callback_flag = true;
}

int main(int argc, char**argv){
    ros::init(argc, argv, "publish_map");
    ros::NodeHandle nh;
    ros::Subscriber subLaserOdometry2 = nh.subscribe<nav_msgs::Odometry>
            ("/aft_mapped_to_init", 5, OdomHandler);
    ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>
            ("/velodyne_points", 2, PointCloudHandler);

    ros::Publisher pubLaserOdometry2 = nh.advertise<geometry_msgs::TransformStamped>
            ("/odom_transform", 5);

    ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>
            ("/velodyne_points_2", 2);
    ros::Rate rate(10);
    bool status = ros::ok();
    current_time = ros::Time::now();
    while (status) {
        if(callback_flag) {
            laserCloudFullRes3.header.frame_id = "/velodyne";
            pubLaserCloudFullRes.publish(laserCloudFullRes3);
            //send the transform
            pubLaserOdometry2.publish(odom_trans);
            callback_flag = false;
        }
        ros::spinOnce();
        status = ros::ok();
        rate.sleep();
    }
    return 0;
}