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

         std::cout << "Predicted robot pose: (" << state.get_data_entry(0, 0) << ", " 
                 << state.get_data_entry(1, 0) << ")" << std::endl;
    
}

void EKFSLAM::simulate() {
    std::cout << "Starting SLAM simulation in 3x3 grid world" << std::endl;

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

        printState();
    }

    std::cout << "\n=== SIMULATION COMPLETE ===" << std::endl;
    std::cout << "Final covariance matrix:" << std::endl;
    covariance.print();

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