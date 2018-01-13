//
// Created by huyh on 18-1-4.
//

#include <iostream>
#include <time.h>
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
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>

using namespace std;
using namespace g2o;

int main(int argc, char** argv){

    SparseOptimizer* graph=new SparseOptimizer();
    //配置优化方法
    BlockSolverX::LinearSolverType * linearSolver = new LinearSolverCSparse<BlockSolverX::PoseMatrixType>();
    BlockSolverX* blockSolver = new BlockSolverX(linearSolver);
    OptimizationAlgorithmLevenberg* optimizationAlgorithm = new OptimizationAlgorithmLevenberg(blockSolver);
    graph->setAlgorithm(optimizationAlgorithm);

    ifstream fin( argv[1] );
    if ( !fin )
    {
        cout<<"file "<<argv[1]<<" does not exist."<<endl;
        return 1;
    }
    int vertexCnt = 0, edgeCnt = 0; // 顶点和边的数量
    while ( !fin.eof() )
    {
        string name;
        fin>>name;
        if ( name == "VERTEX_SE3:QUAT" )
        {
            // SE3 顶点
            g2o::VertexSE3* v = new g2o::VertexSE3();
            int index = 0;
            fin>>index;
            v->setId( index );
            v->read(fin);
            graph->addVertex(v);
            vertexCnt++;
            if ( index==0 )
                v->setFixed(true);
        }
        else if ( name=="EDGE_SE3:QUAT" )
        {
            // SE3-SE3 边
            g2o::EdgeSE3* e = new g2o::EdgeSE3();
            int idx1, idx2;     // 关联的两个顶点
            fin>>idx1>>idx2;
            e->setId( edgeCnt++ );
            e->setVertex( 0, graph->vertices()[idx1] );
            e->setVertex( 1, graph->vertices()[idx2] );
            e->read(fin);
            graph->addEdge(e);
        }
        if ( !fin.good() ) break;
    }
    double start = clock();
    graph->initializeOptimization();
    //优化迭代次数
    graph->optimize(10);
    double finish = clock();
    double exe_time = (double)(finish-start)/CLOCKS_PER_SEC;
    cout<<"Optimize "<<edgeCnt<<" edges cost "<<exe_time<<"seconds"<<endl;
    graph->save("graph_optimized.g2o");
    return 0;
}
