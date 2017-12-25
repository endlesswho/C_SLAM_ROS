#include <iostream>
#include "./TrackTarget.h"

Eigen::Vector4f TrackTarget::predict() {
  float lastX = state(0), lastY = state(1);
  state = F * state;
  P = F * P * F.transpose() + Q;
  Eigen::Vector4f tmp;
  tmp << state(0),
         state(1),
         massZ,
         0;
  return tmp;
}

void TrackTarget::update(Matrix<float,6,1> obj_center) {

  Matrix<float, 2, 1> measurement, y;
  Matrix<float, 4, 2> K;
  Matrix2f S;
  measurement << obj_center(0),
                 obj_center(1);
  Matrix4f I;
  I.setIdentity();
  y = measurement - H * state;
  S = H * P * H.transpose() + R;
  K = P * H.transpose() * S.inverse();
  state += K * y;
  P = (I - K * H) * P;
  Matrix<float,6,1> tmp;
     tmp<<state(0),
          state(1),
          massZ,
          obj_center(3),
          obj_center(4),
          obj_center(5);
  trajectory.push_back(tmp);
}
