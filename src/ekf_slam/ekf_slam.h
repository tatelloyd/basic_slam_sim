#ifndef EKF_SLAM_H
#define EKF_SLAM_H

#include "matrix/matrix.h"
#include "landmark/landmark.h"

class EKFSLAM {
    public:
    
        EKFSLAM();
        void predict(double dx, double dy);
        void updateWithLandmark(double measured_x, double measured_y, int landmark_id);
        void printState() const;
        void simulate();


    private:
        Matrix state;           // Robot pose + landmark positions [x_r, y_r, vx_r, vy_r, x_l1, y_l1, ...]
        Matrix covariance;      // State covariance matrix
        std::vector<Landmark> landmarks;
        int robot_state_size = 2;  // [x, y]

        void addNewLandmark(double measured_x, double measured_y, int landmark_id);
        void updateExistingLandmark(double measured_x, double measured_y, int landmark_index);
};

#endif //EKF_SLAM_H