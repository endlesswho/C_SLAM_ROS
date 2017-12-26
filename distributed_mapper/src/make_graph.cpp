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
// g2o
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
// #include <g2o/solvers/cholmod/linear_solver_cholmod.h>
// gtsam
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


//TODO 问题1：保存的图从LOAM中读取得到，此处默认起始位置和终止位置是同一个点，这里的代码出现的问题主要是GPS的方位角存在一定的问题，起始点的伸展方向存在一定问题，解决方法就：重新采集数据进行测试。

using namespace std;
using namespace g2o;

bool callback_flag = false;
bool init_flag = false;
bool gps_init_flag = true;
bool basic_trans_init_flag = false;
bool continue_flag = false;
bool cloudCBRes = false;
const int ROBOT_A_IDX = 550;
int ROBOT_B_IDX = 0;

float transformAftMapped[7] = {0,0,0,0,0,0,0};
float GPS_transformation[7] = {0,0,0,0,0,0,0};

double latitude_init, longitude_init;

Eigen::Isometry3d basic_trans;
Eigen::Isometry3d step;
Eigen::Isometry3d step_last;
Eigen::Isometry3d ROBOT_A_GPS;
Eigen::Isometry3d ROBOT_B_GPS;

// should be done as singleton
class UniqueId
{
public:
    UniqueId():unique_id(1){}
    int getUniqueId()
    {
        return unique_id++;
    }
private:
    int unique_id;
};
static UniqueId uniqueId;

void addVertex(g2o::OptimizableGraph* graph_ptr, int id, Eigen::Isometry3d pose){
    VertexSE3 *v = new VertexSE3();
    v->setEstimate(pose);
    v->setId(id);
    if(id==0){
        v->setFixed(true);
    } else{
        v->setFixed(false);
    }
    graph_ptr->addVertex(v);
}

void addEdge(g2o::OptimizableGraph* graph_ptr, int64_t id_a, int id_b, Eigen::Isometry3d a_T_b){
    EdgeSE3 *edge = new EdgeSE3();
    cout<<"id_a:"<<id_a<<" id_b:"<<id_b<<endl;
    edge->vertices() [0] = graph_ptr->vertex( id_a );
    edge->vertices() [1] = graph_ptr->vertex( id_b );
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
    edge->setInformation( information );
    edge->setMeasurement( a_T_b );
    graph_ptr->addEdge(edge);
}

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

    step = Eigen::Quaterniond(transformAftMapped[0], transformAftMapped[1], transformAftMapped[2], transformAftMapped[3]).normalized().toRotationMatrix();
    step.translation() = Eigen::Vector3d(transformAftMapped[4], transformAftMapped[5], transformAftMapped[6]);

    callback_flag = true;
}

void getGPSHandler(const sensor_msgs::NavSatFix::ConstPtr& gps) {
    if(gps_init_flag){
        longitude_init = gps->longitude;
        latitude_init = gps->latitude;
        gps_init_flag = false;
    }
    // calc the distacnce of the gps node to the init points: delta_x and delta_y
    double delta_longitude = (gps->longitude - longitude_init)*M_PI/180.0;
    double delta_latitude = (gps->latitude - latitude_init)*M_PI/180.0;
    double media_latitude = (gps->latitude + latitude_init)*M_PI/360.0;
    double delta_x = delta_longitude*6367000*cos(media_latitude);
    double delta_y = 6367000*delta_latitude;

    GPS_transformation[4] = delta_x;
    GPS_transformation[5] = delta_y;
    GPS_transformation[6] = 0;
}

void getQuatHandler(const geometry_msgs::QuaternionStamped::ConstPtr& quat) {
    geometry_msgs::Quaternion geoQuat = quat->quaternion;
    GPS_transformation[0] = geoQuat.w;
    GPS_transformation[1] = geoQuat.x;
    GPS_transformation[2] = geoQuat.y;
    GPS_transformation[3] = geoQuat.z;
    cloudCBRes = true;
}

int main(int argc, char** argv) {

    SparseOptimizer* graph=new SparseOptimizer();

//    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,6>> Block;  // 6x6 BlockSolver
//    Block::LinearSolverType* linearSolver = new g2o::LinearSolverCholmod<Block::PoseMatrixType>(); // 线性方程求解器
//    Block* solver_ptr = new Block( linearSolver );      // 矩阵块求解器
//    // 梯度下降方法，从GN, LM, DogLeg 中选
//    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( solver_ptr );
//    graph->setAlgorithm(solver);

    ros::init(argc, argv, "make_graph");

    ros::NodeHandle nh;
    // subscribe a topic
    ros::Subscriber subOdomAftIntegrated = nh.subscribe<nav_msgs::Odometry>
            ("/integrated_to_init", 10, getPoseHandler);
    ros::Subscriber subGPS = nh.subscribe<sensor_msgs::NavSatFix>
            ("/sbg_ellipse/navsat/fix", 5, getGPSHandler);
    ros::Subscriber subQuat = nh.subscribe<geometry_msgs::QuaternionStamped>
            ("/sbg_ellipse/ekf/quat", 5, getQuatHandler);
    ros::Rate rate(10);
    bool status = ros::ok();
    int64_t tmp  = 0;
    while(status){
        if(cloudCBRes&&(!basic_trans_init_flag)){
            // Run Once Only
            basic_trans = Eigen::Quaterniond(GPS_transformation[0], GPS_transformation[1], GPS_transformation[2], GPS_transformation[3]).normalized().toRotationMatrix();
            basic_trans.translation() = Eigen::Vector3d(GPS_transformation[4], GPS_transformation[5], GPS_transformation[6]);
            basic_trans_init_flag = true;
            cout<<"basic_trans"<<endl;
            cout<<basic_trans.matrix()<<endl;
            addVertex(graph, 0, Eigen::Isometry3d::Identity() );
        }
       // add Vertex to the graph
        if(callback_flag){
            const int id = uniqueId.getUniqueId();
            // gtsam key and symbol
            gtsam::Symbol new_key('a', id);
            cout<<"id:"<<id<<endl;
            cout<<"key:"<<new_key.key()<<endl;

            if(id<=ROBOT_A_IDX){
                addVertex(graph, id, step);
                if(init_flag){
                    // add edge to the graph
                    Eigen::Isometry3d a_T_b = step_last.inverse() * step;
                    addEdge(graph, new_key.key(), id-1, a_T_b);
                } else{
                    Eigen::Isometry3d a_T_b = basic_trans.inverse() * step;
                    addEdge(graph, id, id-1, a_T_b);
                }
                step_last = step;
                init_flag = true;
                callback_flag = false;
                // store robot_a_gps value
                ROBOT_A_GPS = Eigen::Quaterniond(GPS_transformation[0], GPS_transformation[1], GPS_transformation[2], GPS_transformation[3]).normalized().toRotationMatrix();
                ROBOT_A_GPS.translation() = Eigen::Vector3d(GPS_transformation[4], GPS_transformation[5], GPS_transformation[6]);
                cout<<"ROBOT_A_GPS"<<endl;
                cout<<ROBOT_A_GPS.matrix()<<endl;
            }else{
                Eigen::Isometry3d trans_B;
                if(init_flag&&(step(0,3)<=0.2&&step(1,3)<=0.2&&step(2,3)<=0.2)){
                    continue_flag = true;
                }
                if(continue_flag){
                    addVertex(graph, id, step);

                    if (init_flag){
                        trans_B = Eigen::Quaterniond(GPS_transformation[0], GPS_transformation[1], GPS_transformation[2], GPS_transformation[3]).normalized().toRotationMatrix();
                        trans_B.translation() = Eigen::Vector3d(GPS_transformation[4], GPS_transformation[5], GPS_transformation[6]);
                        step_last = trans_B;
                        // Eigen::Isometry3d a_T_b = basic_trans.inverse() * step_last;
                        Eigen::Isometry3d a_T_b = step_last.inverse() * Eigen::Isometry3d::Identity();
                        cout<<"step:"<<endl;
                        cout<<step.matrix()<<endl;
                        cout<<"step_last:"<<endl;
                        cout<<step_last.matrix()<<endl;
                        addEdge(graph, id, 0, a_T_b);
//                        addEdge(graph, id, 0, Eigen::Isometry3d::Identity());
                    }else{
                        // add edge to the graph
                        Eigen::Isometry3d a_T_b = step_last.inverse() * step;
                        cout<<"step:"<<endl;
                         cout<<step.matrix()<<endl;
                        cout<<"step_last:"<<endl;
                        cout<<step_last.matrix()<<endl;
                        addEdge(graph, id, id-1, a_T_b);
                    }

                    ROBOT_B_IDX = id;

                    // store robot_b_gps value
                    ROBOT_B_GPS = Eigen::Quaterniond(GPS_transformation[0], GPS_transformation[1], GPS_transformation[2], GPS_transformation[3]).normalized().toRotationMatrix();
                    ROBOT_B_GPS.translation() = Eigen::Vector3d(GPS_transformation[4], GPS_transformation[5], GPS_transformation[6]);
                    cout<<"ROBOT_B_GPS"<<endl;
                    cout<<ROBOT_B_GPS.matrix()<<endl;

                    step_last = step;
                    init_flag = false;
                }
                callback_flag = false;
            }
        }
        ros::spinOnce();
        rate.sleep();
        status = ros::ok();
    }

    //add last edge

    Eigen::Isometry3d a_T_b = ROBOT_A_GPS.inverse()*ROBOT_B_GPS;
    cout<<"a_T_b"<<endl;
    cout<<a_T_b.matrix()<<endl;
    a_T_b = Eigen::Isometry3d::Identity();
    addEdge(graph, ROBOT_B_IDX, ROBOT_A_IDX, a_T_b);
    graph->initializeOptimization();
    graph->save("graph.g2o");
    return 0;
}
