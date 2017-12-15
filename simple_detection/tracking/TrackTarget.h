#ifndef TRACKTARGET_H
#define TRACKTARGET_H
#include <Eigen/Eigen>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

class TrackTarget {
public:

    vector<Matrix<float,6,1> > trajectory;
    Matrix<float, 4, 1> state;
    float massZ;

    TrackTarget(Matrix<float,6,1> center_obj) {
        F << 1, 0, 0.1, 0,
                0, 1, 0, 0.1,
                0, 0, 1, 0,
                0, 0, 0, 1;
        P << 0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, 1000, 0,
                0, 0, 0, 1000;
        H << 1, 0, 0, 0,
                0, 1, 0, 0;
        R.setIdentity();
        Q.setIdentity();
        Q *= 0.1;
        massZ = center_obj(2);
        state << center_obj(0),
                center_obj(1),
                0,
                0;
        Matrix<float,6,1> tmp;
        tmp << state(0),
                state(1),
                massZ,
                center_obj(3),
                center_obj(4),
                center_obj(5);
        trajectory.push_back(tmp);
    };

    Eigen::Vector4f predict();

    void update(Matrix<float,6,1> obj_center);

private:
    Matrix4f F;
    Matrix4f P;
    Matrix4f Q;
    Matrix2f R;
    Matrix<float, 2, 4> H;
};

#endif
