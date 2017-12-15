#ifndef TRACKMANAGER_H
#define TRACKMANAGER_H

#include "./assignment.h"
#include "./TrackTarget.h"
#include <Eigen/Dense>
#include <algorithm>
#include <math.h>
#include <fstream>
#include <stdlib.h>

using namespace Eigen;
using namespace std;

class TrackManager {
public:
    vector<TrackTarget> targets;

    vector<TrackTarget> disappearedTargets;

    TrackManager(std::vector< Matrix<float,6,1> > seq_center);

    TrackManager();

    vector<Eigen::Matrix<float, 7, 1> > update(std::vector< Matrix<float,6,1> > seq_center);

    void addTarget(Matrix<float,6,1> obj_center);

    void associate(int *&assignment, std::vector< Matrix<float,6,1> > seq_center);

    float calcScore(Eigen::Vector4f predict, Eigen::Vector4f observe);

    Eigen::Matrix<float, 6, 1> getTracks();

};

#endif
