#include "ekf_slam.h"

EKFSLAM :: EKFSLAM() : state(2, 1), covariance(2, 2) {
        // Initialize robot state at origin
        state.set_data_entry(0, 0, 0.0); // x
        state.set_data_entry(1, 0, 0.0); // y  
        
        // Initialize covariance with small uncertainty
        covariance.set_data_entry(0, 0, 0.1);// x uncertainty
        covariance.set_data_entry(1, 1, 0.1);; // y uncertainty
        
        std::cout << "EKF-SLAM initialized with 2x1 state vector" << std::endl;
};

void EKFSLAM::predict(double dx, double dy) {
    std::cout << "\n=== PREDICTION STEP ===" << std::endl;
        
        // Simple motion model: x_k+1 = x_k + dx, y_k+1 = y_k + dy
        double x = state.get_data_entry(0,0); 
        double y = state.get_data_entry(1,0); 
        
        // Predict new state
        state.set_data_entry(0, 0, state.get_data_entry(0,0) + dx);
        state.set_data_entry(1, 0, state.get_data_entry(1,0) + dy);
        
        // Jacobian of motion model (A matrix) - 2x2 for robot state
        Matrix A(robot_state_size, robot_state_size);
        A.set_data_entry(0, 0, 1.0);
        A.set_data_entry(1, 1, 1.0);

        // Expand A matrix for full state (including landmarks)
        int full_state_size = robot_state_size + landmarks.size() * 2;
        Matrix A_full(full_state_size, full_state_size);
        
        // Copy robot state transition
        for (int i = 0; i < robot_state_size; i++) {
            for (int j = 0; j < robot_state_size; j++) {
                A_full.set_data_entry(i, j, A.get_data_entry(i, j));
            }
        }
        
        // Landmarks don't move (identity for landmark states)
        for (int i = robot_state_size; i < full_state_size; i++) {
            A_full.set_data_entry(i, i, 1.0);
        }
        
        // Predict covariance: P = A*P*A^T + Q
        // For simplicity, we're ignoring process noise Q as you mentioned
        Matrix A_T = A_full.transpose();
        covariance = A_full * covariance * A_T;

         std::cout << "Predicted robot pose: (" << state.get_data_entry(0, 0) << ", " 
                 << state.get_data_entry(1, 0) << ")" << std::endl;
    
}

void EKFSLAM::updateWithLandmark(double measured_x, double measured_y, int landmark_id) {
    std::cout << "\n=== UPDATE STEP ===" << std::endl;
    std::cout << "Observing landmark " << landmark_id 
                << " at (" << measured_x << ", " << measured_y << ")" << std::endl;
    
    // Check if this is a new landmark
    bool is_new_landmark = true;
    int landmark_index = -1;
    
    for (size_t i = 0; i < landmarks.size(); i++) {
        if (landmarks[i].id == landmark_id) {
            is_new_landmark = false;
            landmark_index = i;
            break;
        }
    }
    
    if (is_new_landmark) {
        addNewLandmark(measured_x, measured_y, landmark_id);
    } else {
        updateExistingLandmark(measured_x, measured_y, landmark_index);
    }
}

void EKFSLAM::simulate() {
    std::cout << "Starting SLAM simulation in 3x3 grid world" << std::endl;
    std::cout << "Landmark at center: (1.5, 1.5)" << std::endl;

    // Move robot in a square around the landmark
    std::vector<std::pair<double, double>> moves = {
        {1.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}, {0.0, 1.0},
        {-1.0, 0.0}, {-1.0, 0.0}, {0.0, -1.0}, {0.0, -1.0}
    };

    for (size_t i = 0; i < moves.size(); i++) {
        std::cout << "\n--- Move " << i + 1 
        << ": dx=" << moves[i].first << ", dy=" << moves[i].second << " ---" << std::endl;
            
        // Predict step
        predict(moves[i].first, moves[i].second);

        // Simulate observing the landmark at (1.5, 1.5)
        double landmark_global_x = 1.5;
        double landmark_global_y = 1.5;
        double robot_x = state.get_data_entry(0, 0);
        double robot_y = state.get_data_entry(1, 0);
        
        // Convert to relative measurement
        double measured_x = landmark_global_x - robot_x;
        double measured_y = landmark_global_y - robot_y;
        
        // Only observe if reasonably close (within sensor range)
        double distance = sqrt(measured_x*measured_x + measured_y*measured_y);
        if (distance < 2.0) {
            updateWithLandmark(measured_x, measured_y, 1);
        }

        printState();
    }

    std::cout << "\n=== SIMULATION COMPLETE ===" << std::endl;
    std::cout << "Final covariance matrix:" << std::endl;
    covariance.print();

}

void EKFSLAM::addNewLandmark(double measured_x, double measured_y, int landmark_id) {
    std::cout << "Adding new landmark " << landmark_id << std::endl;
    
    // Convert measurement to global coordinates
    double robot_x = state.get_data_entry(0, 0);
    double robot_y = state.get_data_entry(1, 0);
    
    double global_x = robot_x + measured_x;
    double global_y = robot_y + measured_y;
    
    // Add landmark to our list
    Landmark new_landmark;
    new_landmark.x = global_x;
    new_landmark.y = global_y;
    new_landmark.id = landmark_id;
    new_landmark.observed = true;
    landmarks.push_back(new_landmark);
    
    // Expand state vector
    Matrix new_state(robot_state_size + landmarks.size() * 2, 1);
    for (int i = 0; i < state.get_rows(); i++) {
        new_state.set_data_entry(i, 0, state.get_data_entry(i, 0));
    }

    new_state.set_data_entry(state.get_rows(), 0, global_x);
    new_state.set_data_entry(state.get_rows() + 1, 0, global_y);
    state = new_state;
    
    // Expand covariance matrix
    int new_size = robot_state_size + landmarks.size() * 2;
    Matrix new_covariance(new_size, new_size);
    
    // Copy existing covariance
    for (int i = 0; i < covariance.get_rows(); i++) {
        for (int j = 0; j < covariance.get_cols(); j++) {
            new_covariance.set_data_entry(i, j, covariance.get_data_entry(i, j));
        }
    }
    
    // Initialize new landmark uncertainty (high uncertainty for new landmarks)
    new_covariance.set_data_entry(new_size-2, new_size-2, 1.0); // x uncertainty
    new_covariance.set_data_entry(new_size-1, new_size-1, 1.0); // y uncertainty
    
    covariance = new_covariance;
    
    std::cout << "Landmark added at global position: (" 
                << global_x << ", " << global_y << ")" << std::endl;
}
    
void EKFSLAM::updateExistingLandmark(double measured_x, double measured_y, int landmark_index) {
    std::cout << "Updating existing landmark " << landmarks[landmark_index].id << std::endl;
    
    // Get robot and landmark positions
    double robot_x = state.get_data_entry(0, 0);
    double robot_y = state.get_data_entry(1, 0);
    
    int lm_state_idx = robot_state_size + landmark_index * 2;
    double landmark_x = state.get_data_entry(lm_state_idx, 0);
    double landmark_y = state.get_data_entry(lm_state_idx + 1, 0);
    
    // Predicted measurement (what we expect to see)
    double predicted_x = landmark_x - robot_x;
    double predicted_y = landmark_y - robot_y;
    
    // Innovation (difference between actual and predicted measurement)
    Matrix innovation(2, 1);
    innovation.set_data_entry(0, 0, measured_x - predicted_x);
    innovation.set_data_entry(1, 0, measured_y - predicted_y);
    
    // Measurement Jacobian H (2x2 for robot state only, since landmarks don't affect measurement directly)
    Matrix H(2, state.get_rows());
    H.set_data_entry(0, 0, -1.0); // ∂(lm_x - robot_x)/∂robot_x
    H.set_data_entry(1, 1, -1.0); // ∂(lm_y - robot_y)/∂robot_y
    H.set_data_entry(0, lm_state_idx, 1.0);     // ∂(lm_x - robot_x)/∂lm_x
    H.set_data_entry(1, lm_state_idx + 1, 1.0); // ∂(lm_y - robot_y)/∂lm_y
    
    // Measurement noise covariance R (simplified, no noise in simulation)
    Matrix R(2, 2);
    R.set_data_entry(0, 0, 0.01);
    R.set_data_entry(1, 1, 0.01);
    
    // Kalman gain calculation: K = P*H^T*(H*P*H^T + R)^-1
    Matrix H_T = H.transpose();
    Matrix S = H * covariance * H_T + R;
    Matrix S_inv = S.inverse();
    Matrix K = covariance * H_T * S_inv;
    
    // State update: x = x + K*innovation
    Matrix state_update = K * innovation;
    state = state + state_update;
    
    // Covariance update: P = (I - K*H)*P
    Matrix I(state.get_rows(), state.get_rows());
    for (int i = 0; i < state.get_rows(); i++) {
        I.set_data_entry(i, i, 1.0);
    }
    Matrix KH = K * H;
    covariance = (I - KH) * covariance;
    
    std::cout << "Innovation: (" << innovation.get_data_entry(0, 0) 
                << ", " << innovation.get_data_entry(1, 0) << ")" << std::endl;
    std::cout << "Updated robot pose: (" << state.get_data_entry(0, 0)
                << ", " << state.get_data_entry(1, 0) << ")" << std::endl;
}

   

void EKFSLAM::printState() const {
        std::cout << "\n=== CURRENT STATE ===" << std::endl;
        std::cout << "Robot pose: (" << state.get_data_entry(0, 0) << ", " << state.get_data_entry(1, 0) << ")" << std::endl;
        
        std::cout << "Landmarks:" << std::endl;
        for (size_t i = 0; i < landmarks.size(); i++) {
            int idx = robot_state_size + i * 2;
            std::cout << "  ID " << landmarks[i].id << ": (" 
                     << state.get_data_entry(idx, 0) << ", " << state.get_data_entry(idx + 1, 0) << ")" << std::endl;
        }
        
        std::cout << "State vector size: " << state.get_rows() << "x" << state.get_cols() << std::endl;
        std::cout << "Covariance size: " << covariance.get_rows() << "x" << covariance.get_cols() << std::endl;
    }