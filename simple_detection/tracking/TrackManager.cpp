#include "./TrackManager.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

AssignmentProblemSolver SolveThisProblem;

TrackManager::TrackManager(std::vector< Matrix<float,6,1> > seq_center) {
    int size = seq_center.size();
    for (int i = 0; i < size; i++) {
        addTarget(seq_center[i]);
    }
}

TrackManager::TrackManager() {
}

void TrackManager::addTarget(Matrix<float,6,1> obj_center) {
    targets.push_back(TrackTarget(obj_center));
}

void TrackManager::associate(int * &assignment, std::vector< Matrix<float,6,1> > seq_center) {
    int targetNum = targets.size();
    int observeNum = seq_center.size();
    // const float GATING_DIST = 0.7;
    const float GATING_DIST = 1;

    // calc cost matrix
    double* costMatrix = new double[targetNum*observeNum];

    // each row correspond to one observations(i) ; col: targets (j)
    int count = 0;
    for (int j = 0; j < targetNum; j++) {
        for (int i = 0; i < observeNum; i++) {
//            Matrix<float,6,1> last_results = targets[j].trajectory.back();
//            Eigen::Vector4f predict (last_results(0),last_results(1),last_results(2),0);
             Eigen::Vector4f predict = targets[j].predict();
            Eigen::Vector4f observe;
            observe<<seq_center[i](0),
                    seq_center[i](1),
                    seq_center[i](2),
                    0;

            float dist = sqrt(pow(predict(0)-observe(0),2)+pow(predict(1)-observe(1),2));
            float gating;
            float score;
            if (targets[j].trajectory.size() <= 1) {
                gating = GATING_DIST;
                score = pow(2, -dist);
            } else {
                gating = GATING_DIST;
                score = calcScore(predict, observe);
            }
            if (dist > gating) {
                costMatrix[count] = ASSIGNMENT_INF;
            } else {
                //should be normalized to 0-1
                costMatrix[count] = score;
            }
            count++;
        }
    }

    // log costMatrix
  cout << "costMatrix:" << endl;
  for (int i = 0; i < observeNum; i++) {
    for (int j = 0; j < targetNum; j++) {
      cout << costMatrix[i+j*observeNum] << " ";
    }
    cout << endl;
  }


    double* cost = new double[1];
    SolveThisProblem.assignmentoptimal(assignment, cost, costMatrix, observeNum, targetNum);
    // log assignment
//  cout << "assignment:" << endl;
//  for (int i = 0; i < observeNum; i++) {
//    cout << assignment[i] << endl;
//  }
    delete[] costMatrix;
}

vector<Eigen::Matrix<float, 7, 1> > TrackManager::update(vector< Matrix<float,6,1> > seq_center) {
    vector<Eigen::Matrix<float, 7, 1> > current_objs_idx;
    Eigen::Matrix<float, 7, 1> current_obj_idx;
    int tNum = targets.size();
    int cNum = seq_center.size();
    if (!tNum) {
        for (int i = 0; i < seq_center.size(); i++) {
            addTarget(seq_center[i]);
            current_obj_idx << seq_center[i],
                    i;
            current_objs_idx.push_back(current_obj_idx);
        }
        return current_objs_idx;
    } else if (cNum) {
        bool * is_target_matched = new bool[tNum];
        int * assignment = new int[cNum];
        // int * assignment;
        for (int i = 0; i < tNum; i++) {
            is_target_matched[i] = false;
        }

        associate(assignment, seq_center);

        for (int i = 0; i < cNum; i++) {
            int assignTo = assignment[i];
            if (assignTo < 0) {
                // new observation
                addTarget(seq_center[i]);
                current_obj_idx<<seq_center[i],
                        targets.size();
                current_objs_idx.push_back(current_obj_idx);
            } else {

                is_target_matched[assignTo] = true;
                targets[assignTo].update(seq_center[i]);
                current_obj_idx<<seq_center[i],
                        assignTo;
                current_objs_idx.push_back(current_obj_idx);
            }
        }
        // remove false alarm or store disappeared target
//        for (int i = tNum - 1; i > 0; i--) {
//            if (!is_target_matched[i]) {
//                if (targets[i].trajectory.size() > 5) {
//                    disappearedTargets.push_back(targets[i]);
//                }
//                targets.erase(targets.begin()+i);
//            }
//        }
        delete[] is_target_matched;
        delete[] assignment;
        return current_objs_idx;
    }
}

float TrackManager::calcScore(Eigen::Vector4f predict, Eigen::Vector4f observe) {
    float score = sqrt(pow(predict(0)-observe(0),2)+pow(predict(1)-observe(1),2));
    return score;
}

Eigen::Matrix<float, 6, 1> TrackManager::getTracks(){
    Matrix<float, 6, 1> current_objs;
    TrackTarget target = targets.back();
    current_objs = target.trajectory.back();
}

