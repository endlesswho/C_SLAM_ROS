//
// Created by huyh on 17-12-23.
//

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
// ROS
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/NavSatFix.h>

//Dist Mapper Optimization
#include <DistributedMapperUtils.h>
#include <MultiRobotUtils.h>
#include <BetweenChordalFactor.h>

#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */


using namespace std;
using namespace gtsam;
using namespace distributed_mapper;
//using namespace multirobot_util;

float transformAftMapped[7] = {0,0,0,0,0,0,0};
bool callback_flag = false;
size_t maxIter = 1000; // Maximum number of iterations of optimizer
DistributedMapper::UpdateType updateType = DistributedMapper::incUpdate; // updateType differetiates between Distributed Jacobi/Jacobi OverRelaxation (postUpdate) and Gauss-Seidel/Successive OverRelaxation (incUpdate)
double rotationEstimateChangeThreshold = 1e-1; // Difference between rotation estimate provides an early stopping condition
double poseEstimateChangeThreshold = 1e-1; // Difference between pose estimate provides an early stopping condition
double gamma_value = 1.0f; // Gamma value for over relaxation methods
bool useFlaggedInit = true; // to use flagged initialization or not
bool useLandmarks = false; // to use landmarks or not

Eigen::Isometry3d current_pose;

void getPoseHandler(const nav_msgs::Odometry::ConstPtr& odomAftMapped){

    geometry_msgs::Quaternion geoQuat = odomAftMapped->pose.pose.orientation;

    transformAftMapped[0] = geoQuat.w;
    transformAftMapped[1] = geoQuat.x;
    transformAftMapped[2] = geoQuat.y;
    transformAftMapped[3] = geoQuat.z;
    //cout<<"Quat::"<<geoQuat.x<<" "<<geoQuat.y<<" "<<geoQuat.z<<" "<<geoQuat.w<<endl;

    transformAftMapped[4] = odomAftMapped->pose.pose.position.x;
    transformAftMapped[5] = odomAftMapped->pose.pose.position.y;
    transformAftMapped[6] = odomAftMapped->pose.pose.position.z;

    current_pose = Eigen::Quaterniond(transformAftMapped[0], transformAftMapped[1], transformAftMapped[2], transformAftMapped[3]).normalized().toRotationMatrix();
    current_pose.translation() = Eigen::Vector3d(transformAftMapped[4], transformAftMapped[5], transformAftMapped[6]);

    callback_flag = true;
}

/**
 * @brief main function
 */
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "distributed_optimization_ros");

    ros::NodeHandle nh;
    // subscribe a topic
    ros::Subscriber subOdomAftIntegrated = nh.subscribe<nav_msgs::Odometry>
            ("/integrated_to_init", 10, getPoseHandler);
    ros::Rate rate(10);
    bool status = ros::ok();
    int64_t tmp  = 0;
    // Vector of distributed optimizers, one for each robot
    vector< boost::shared_ptr<DistributedMapper> > distMappers;
    while(status){
        if(callback_flag){

        }
        ros::spinOnce();
    }

    // Vectors containing logs 存储函数调用日志
    vector < Values > rotationTrace;
    vector < Values > poseTrace;
    vector < Values > subgraphRotationTrace;
    vector < Values > subgraphPoseTrace;
    vector < VectorValues > rotationVectorValuesTrace;
    vector< Values > estimates =  distributedOptimizer(distMappers, maxIter, updateType, gamma_value,
                                                       rotationEstimateChangeThreshold, poseEstimateChangeThreshold, useFlaggedInit, useLandmarks,
                                                       false, rotationTrace, poseTrace, subgraphRotationTrace, subgraphPoseTrace, rotationVectorValuesTrace);
    return 0;
}

