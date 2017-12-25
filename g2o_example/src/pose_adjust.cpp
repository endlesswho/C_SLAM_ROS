
#include <Eigen/Core>

// ROS
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/NavSatFix.h>

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include <g2o/types/slam3d/types_slam3d.h> //顶点类型

using namespace std;
using namespace g2o;

int idx = 0;
nav_msgs::Odometry gps_Odometry;
double latitude;
double longitude;
bool QUAT_FLAG = false;
bool cloudCBRes = false;
bool init_flag = false;
bool find_loop = false;
bool callback_flag = false;

Eigen::Isometry3d step;
Eigen::Isometry3d step_last;

float transformAftMapped[7] = {0,0,0,0,0,0,0};

const int maxIterations = 10;

// should be done as singleton
class UniqueId
{
public:
    UniqueId():unique_id(0){}
    int getUniqueId()
    {
        return unique_id++;
    }
private:
    int unique_id;
};
static UniqueId uniqueId;


Eigen::Isometry3d isometryFromArray7D( float* v)
{
    Eigen::Isometry3d result;

    result = Eigen::Quaterniond(v[0], v[1], v[2], v[3]).normalized().toRotationMatrix();
    result.translation() = Eigen::Vector3d(v[4], v[5], v[6]);
    return result;
}

void addEdge(const Eigen::Isometry3d & a_T_b, const int id_a, const int id_b,
             g2o::OptimizableGraph* graph_ptr)
{
    EdgeSE3 *e = new EdgeSE3;
    //EdgePosePose * e = new EdgePosePose;

    // retrieve vertex pointers from graph with id's
    g2o::OptimizableGraph::Vertex * pose_a_vertex
            = dynamic_cast<g2o::OptimizableGraph::Vertex*>
            (graph_ptr->vertices()[id_a]);

    g2o::OptimizableGraph::Vertex * pose_b_vertex
            = dynamic_cast<g2o::OptimizableGraph::Vertex*>
            (graph_ptr->vertices()[id_b]);

    // error check vertices and associate them with the edge
    assert(pose_a_vertex!=NULL);
    assert(pose_a_vertex->dimension() == 6);
    e->vertices()[0] = pose_a_vertex;

    assert(pose_b_vertex!=NULL);
    assert(pose_b_vertex->dimension() == 6);
    e->vertices()[1] = pose_b_vertex;

    // add information matrix
    Eigen::Matrix<double, 6, 6> Lambda;
    Lambda.setIdentity();

    // set the observation and imformation matrix
    e->setMeasurement(a_T_b);
    e->information() = Lambda;

    // finally add the edge to the graph
    if(!graph_ptr->addEdge(e))
    {
        assert(false);
    }
}

void getPoseHandler(const nav_msgs::Odometry::ConstPtr& odomAftMapped){
    double roll, pitch, yaw;
    geometry_msgs::Quaternion geoQuat = odomAftMapped->pose.pose.orientation;
//    tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);
    transformAftMapped[0] = geoQuat.w;
    transformAftMapped[1] = geoQuat.x;
    transformAftMapped[2] = geoQuat.y;
    transformAftMapped[3] = geoQuat.z;
    cout<<"Quat::"<<geoQuat.x<<" "<<geoQuat.y<<" "<<geoQuat.z<<" "<<geoQuat.w<<endl;

    transformAftMapped[4] = odomAftMapped->pose.pose.position.x;
    transformAftMapped[5] = odomAftMapped->pose.pose.position.y;
    transformAftMapped[6] = odomAftMapped->pose.pose.position.z;

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    // poses
    // std::cout << "Generating Poses...\n";
    step = isometryFromArray7D(transformAftMapped);
    cout<<"step:"<<endl;
    cout<<step.matrix()<<endl;
    callback_flag = true;
}

//#if QUAT_FLAG
//void getGPSHandler(const sensor_msgs::NavSatFix::ConstPtr& gps) {
//    if(init_flag){
//        longitude = gps->longitude;
//        latitude = gps->latitude;
//        init_flag = false;
//    }
//
//    double delta_longitude = (gps->longitude - longitude)*M_PI/180.0;
//    double delta_latitude = (gps->latitude - latitude)*M_PI/180.0;
//    double media_latitude = (gps->latitude + latitude)*M_PI/360.0;
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
//
//void getQuatHandler(const geometry_msgs::QuaternionStamped::ConstPtr& quat) {
//    geometry_msgs::Quaternion Quat = quat->quaternion;
//    gps_Odometry.pose.pose.orientation.x = Quat.x;
//    gps_Odometry.pose.pose.orientation.y = Quat.y;
//    gps_Odometry.pose.pose.orientation.z = Quat.z;
//    gps_Odometry.pose.pose.orientation.w = Quat.w;
//    cloudCBRes = true;
//}
//#endif

int main(int argc, char** argv)
{
    SparseOptimizer* graph=new SparseOptimizer();

    ros::init(argc, argv, "pose_adjust");
    ros::NodeHandle nh;

    // subscribe a topic
    ros::Subscriber subOdomAftIntegrated = nh.subscribe<nav_msgs::Odometry>
            ("/integrated_to_init", 10, getPoseHandler);

//#if QUAT_FLAG
//    ros::Subscriber subGPS = nh.subscribe<sensor_msgs::NavSatFix>
//            ("/sbg_ellipse/navsat/fix", 5, getGPSHandler);
//
//    ros::Subscriber subQuat = nh.subscribe<geometry_msgs::QuaternionStamped>
//            ("/sbg_ellipse/ekf/quat", 5, getQuatHandler);
//#endif

    ros::Rate rate(10);
    bool status = ros::ok();
    while(status){

//        // add Vertex to the graph
        if(callback_flag){
            const int id = uniqueId.getUniqueId();
            VertexSE3 *v = new VertexSE3();
//            v->setEstimate(step);
            v->setEstimate( Eigen::Isometry3d::Identity());
            v->setId(id);
            if(id==0){
                v->setFixed(true);
            } else{
                v->setFixed(false);
            }
            graph->addVertex(v);
            if(init_flag){
                // add edge to the graph
                Eigen::Isometry3d a_T_b = step_last.inverse() * step;
                cout<<"a_T_b:"<<endl;
                cout<<a_T_b.matrix()<<endl;
//                addEdge(a_T_b, id, id-1, &graph);
                g2o::EdgeSE3* edge = new g2o::EdgeSE3();
                edge->setId(id-1);
                edge->vertices() [0] = graph->vertex( id );
                edge->vertices() [1] = graph->vertex( id-1 );
                Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
                information(0,0) = information(1,1) = information(2,2) = 10;
                information(3,3) = information(4,4) = information(5,5) = 10;
                cout<<information.matrix()<<endl;
                edge->setInformation( information );
                edge->setMeasurement( a_T_b );
                graph->addEdge(edge);
                if(id == 200)
                    graph->save("./graph.g2o");
            }
            step_last = step;
            init_flag = true;
        }
        ros::spinOnce();
        rate.sleep();
        status = ros::ok();
    }
//    graph->initializeOptimization();
//    graph->save("./graph.g2o");
    cout<<"graph saved......."<<endl;
    return 0;
}
