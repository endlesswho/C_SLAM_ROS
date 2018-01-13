//
// Created by huyh on 17-12-23.
// make gtsam graph
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
//Boost
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
const int ROBOT_1_IDX = 1300;
DistributedMapper::UpdateType updateType = DistributedMapper::incUpdate; // updateType differetiates between Distributed Jacobi/Jacobi OverRelaxation (postUpdate) and Gauss-Seidel/Successive OverRelaxation (incUpdate)
double rotationEstimateChangeThreshold = 1e-1; // Difference between rotation estimate provides an early stopping condition
double poseEstimateChangeThreshold = 1e-1; // Difference between pose estimate provides an early stopping condition
double gamma_value = 1.0f; // Gamma value for over relaxation methods
bool useFlaggedInit = true; // to use flagged initialization or not
bool useLandmarks = false; // to use landmarks or not
const string tag_vertex = "VERTEX_SE3:QUAT";
const string tag_edge = "EDGE_SE3:QUAT";
Eigen::Isometry3d current_pose;

void addInitial(const gtsam::Symbol &symbol_id, gtsam::Values::shared_ptr initial, double *data){
    //转换到gtsamPose3
    gtsam::Rot3 R= gtsam::Rot3::Quaternion(data[6],data[3],data[4],data[5]);
    gtsam::Point3 t(data[0],data[1],data[2]);
    initial->insert(symbol_id, gtsam::Pose3(R,t));
}

void addFactor(const gtsam::Symbol &symbol_id1, const gtsam::Symbol &symbol_id2, gtsam::NonlinearFactorGraph::shared_ptr graph, double *data, gtsam::Matrix m){
    //转换到gtsamPose3
    gtsam::Rot3 R= gtsam::Rot3::Quaternion(data[6],data[3],data[4],data[5]);
    gtsam::Point3 t(data[0],data[1],data[2]);

    //g2o的信息矩阵和gtsam定义方式不一样，这里需要对其进行修改
    gtsam::Matrix mgtsam = gtsam::I_6x6;
    mgtsam.block<3,3>(0,0) = m.block<3,3>(3,3);
    mgtsam.block<3,3>(3,3) = m.block<3,3>(0,0);
    mgtsam.block<3,3>(0,3) = m.block<3,3>(0,3);
    mgtsam.block<3,3>(3,0) = m.block<3,3>(3,0);
    gtsam::SharedNoiseModel model = gtsam::noiseModel::Gaussian::Information(mgtsam);
    gtsam::NonlinearFactor::shared_ptr factor(new gtsam::BetweenFactor<gtsam::Pose3>(symbol_id1.key(),symbol_id2.key(),gtsam::Pose3(R,t),model));

    graph->push_back(factor);
}

void getPoseHandler(const nav_msgs::Odometry::ConstPtr& odomAftMapped) {

    geometry_msgs::Quaternion geoQuat = odomAftMapped->pose.pose.orientation;

    transformAftMapped[0] = geoQuat.w;
    transformAftMapped[1] = geoQuat.x;
    transformAftMapped[2] = geoQuat.y;
    transformAftMapped[3] = geoQuat.z;
    //cout<<"Quat::"<<geoQuat.x<<" "<<geoQuat.y<<" "<<geoQuat.z<<" "<<geoQuat.w<<endl;

    transformAftMapped[4] = odomAftMapped->pose.pose.position.x;
    transformAftMapped[5] = odomAftMapped->pose.pose.position.y;
    transformAftMapped[6] = odomAftMapped->pose.pose.position.z;

    current_pose = Eigen::Quaterniond(transformAftMapped[0], transformAftMapped[1], transformAftMapped[2],
                                      transformAftMapped[3]).normalized().toRotationMatrix();
    current_pose.translation() = Eigen::Vector3d(transformAftMapped[4], transformAftMapped[5], transformAftMapped[6]);

    callback_flag = true;
}

 /**
 * @brief main function
 */
int main(int argc, char* argv[])
{
    int64_t tmp  = 0;
    int cntVertex = 0, cntEdge = 0;
    gtsam::NonlinearFactorGraph::shared_ptr graph_a(new gtsam::NonlinearFactorGraph);
    gtsam::NonlinearFactorGraph::shared_ptr graph_b(new gtsam::NonlinearFactorGraph);
    // 初始值
    gtsam::Values::shared_ptr initial_a(new gtsam::Values);
    gtsam::Values::shared_ptr initial_b(new gtsam::Values);

    // Vector of distributed optimizers, one for each robot
    vector< boost::shared_ptr<DistributedMapper> > distMappers;
    // Construct a distributed jacobi object with the given robot name
    boost::shared_ptr<DistributedMapper> distMapper(new DistributedMapper('a'));
    ifstream fin("./graph.g2o");
    cout<<"reading g2o file...."<<endl;

    while(!fin.eof()){
        string tag;
        fin>>tag;
        if(tag=="VERTEX_SE3:QUAT"){
            gtsam::Key id;
            fin>>id;
            double data[7];
            for(int i=0; i<7; i++){
                fin>>data[i];
            }
            gtsam::Symbol symbol_id;
            if(id<=ROBOT_1_IDX){
                symbol_id=symbol('a',id);
                addInitial(symbol_id, initial_a, data);
            }

            else{
                symbol_id=symbol('b',id);
                addInitial(symbol_id, initial_b, data);
            }
//            //转换到gtsamPose3
//            gtsam::Rot3 R= gtsam::Rot3::Quaternion(data[6],data[3],data[4],data[5]);
//            gtsam::Point3 t(data[0],data[1],data[2]);
//            initial->insert(symbol_id.key(), gtsam::Pose3(R,t));

        }
        else if (tag =="EDGE_SE3:QUAT"){
            gtsam::Matrix m = gtsam::I_6x6;
            gtsam::Key id1, id2;
            fin>>id1>>id2;
            gtsam::Symbol symbol_id1;
            gtsam::Symbol symbol_id2;
            if(id1<=ROBOT_1_IDX)
                symbol_id1=symbol('a',id1);
            else
                symbol_id1=symbol('b',id1);
            if(id2<=ROBOT_1_IDX)
                symbol_id2=symbol('a',id2);
            else
                symbol_id2=symbol('b',id2);
            double data[7];
            for(int i=0;i<7;i++)
                fin>>data[i];
//            //转换到gtsamPose3
//            gtsam::Rot3 R= gtsam::Rot3::Quaternion(data[6],data[3],data[4],data[5]);
//            gtsam::Point3 t(data[0],data[1],data[2]);

            for(int i=0;i<6;i++){
                for(int j=i;j<6;j++){
                    double mij;
                    fin>>mij;
                    m(i,j)=mij;
                    m(j,i)=mij;
                }
            }
//            //g2o的信息矩阵和gtsam定义方式不一样，这里需要对其进行修改
//            gtsam::Matrix mgtsam = gtsam::I_6x6;
//            mgtsam.block<3,3>(0,0) = m.block<3,3>(3,3);
//            mgtsam.block<3,3>(3,3) = m.block<3,3>(0,0);
//            mgtsam.block<3,3>(0,3) = m.block<3,3>(0,3);
//            mgtsam.block<3,3>(3,0) = m.block<3,3>(3,0);
//
//            gtsam::SharedNoiseModel model = gtsam::noiseModel::Gaussian::Information(mgtsam);
//            gtsam::NonlinearFactor::shared_ptr factor(new gtsam::BetweenFactor<gtsam::Pose3>(symbol_id1.key(),symbol_id2.key(),gtsam::Pose3(R,t),model));
//
//            graph->push_back(factor);
            if(id1<=ROBOT_1_IDX||id2<=ROBOT_1_IDX){
                addFactor(symbol_id1, symbol_id2, graph_a, data, m);
            }
            if(id1>ROBOT_1_IDX||id2>ROBOT_1_IDX){
                addFactor(symbol_id1, symbol_id2, graph_b, data, m);
            }
            cntEdge++;
        }
        if(!fin.good())
            break;

//        cout<<"read total:"<<cntVertex<<" vertices, "<<cntEdge<<" edges."<<endl;
//        //固定第一个定点，在gtsam中相当于添加一个先验因子
//        gtsam::NonlinearFactorGraph graphWithPrior = *graph;
//        gtsam::noiseModel::Diagonal::shared_ptr priorModel = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector(6)<<1e-6,1e-6,1e-6,1e-6,1e-6,1e-6).finished();
//        gtsam::Key firstKey = 0;
//        for(const gtsam::Values::ConstKeyValuePtrPair& key_value: *initial) {
//            cout<<"adding prior to g2o file"<<endl;
//            graphWithPrior.add(gtsam::PriorFactor<gtsam::Pose3>(key_value.key,key_value.value.cast<gtsam::Pose3>(),priorModel));
//            break;
//        }
        // Use between noise or not in optimizePoses
        distMapper->setUseBetweenNoiseFlag(false);
        // Use landmarks
        distMapper->setUseLandmarksFlag(false);
        // Load subgraphs
//            distMapper->loadSubgraphAndCreateSubgraphEdge(graphAndValues);
    }
    writeG2o(*graph_a, *initial_a, "gtsam_graph_a.g2o");
    writeG2o(*graph_b, *initial_b, "gtsam_graph_b.g2o");

    // Vectors containing logs 存储函数调用日志
    vector < Values > rotationTrace;
    vector < Values > poseTrace;
    vector < Values > subgraphRotationTrace;
    vector < Values > subgraphPoseTrace;
    vector < VectorValues > rotationVectorValuesTrace;
//    vector< Values > estimates =  distributedOptimizer(distMappers, maxIter, updateType, gamma_value,
//                                                       rotationEstimateChangeThreshold, poseEstimateChangeThreshold, useFlaggedInit, useLandmarks,
//                                                       true, rotationTrace, poseTrace, subgraphRotationTrace, subgraphPoseTrace, rotationVectorValuesTrace);
    return 0;
}

