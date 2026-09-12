#include <iostream>
#include "Linear_Kalman_Filter/kalmanfilter.h"
#include "Linear_Kalman_Filter/pseudosensors.h"
#include <Eigen/Dense>
#include <vector>
#include <matplot/matplot.h>

// ==================== USER INPUTS (EDIT THIS SECTION) ========================
#define INIT_POS_X           3 // initial positions in m
#define INIT_POS_Y           4
#define INIT_VEL_X           2 // initial velocities in m/s
#define INIT_VEL_Y           1

#define INIT_POS_X_STD       20 // these standard deviations are specifically for initialising P in the Kalman Filter  
#define INIT_POS_Y_STD       20 // (can be larger than the tolerance of the sensor to be conservative; the filter  
#define INIT_VEL_X_STD       10 // will update P accordingly with each epoch)
#define INIT_VEL_Y_STD       10

#define ACCEL_X_STD          1
#define ACCEL_Y_STD          1
#define DT                   1 // time step in seconds

#define GPS_X_STD            3  // standard deviation of GPS measurements in m
#define GPS_Y_STD            3 

// ====== CONVERTING USER INPUTS TO STATE-SPACE REPRESENTATION (MATRICES) ======
Vector2d initialPosition(INIT_POS_X, INIT_POS_Y);

Vector2d initialVelocity(INIT_VEL_X, INIT_VEL_Y);

VectorXd x_0 {{INIT_POS_X, 
               INIT_VEL_X, 
               INIT_POS_Y, 
               INIT_VEL_Y}
}; // initial state conditions

MatrixXd P_0 {{INIT_POS_X_STD*INIT_POS_X_STD, 0, 0, 0},
                {0, INIT_VEL_X_STD*INIT_VEL_X_STD, 0, 0},
                {0, 0, INIT_POS_Y_STD*INIT_POS_Y_STD, 0},
                {0, 0, 0, INIT_VEL_Y_STD*INIT_VEL_Y_STD}
}; // initial state covariances, assume all covariances are 0

MatrixXd F {{1, DT, 0, 0},
            {0, 1, 0, 0},
            {0, 0, 1, DT},
            {0, 0, 0, 1}
}; // state transition matrix

MatrixXd L {{0.5*DT*DT, 0},
            {DT, 0},
            {0, 0.5*DT*DT},
            {0, DT}
}; // noise gain matrix

MatrixXd w {{ACCEL_X_STD, 0}, 
            {0, ACCEL_Y_STD} 
}; // piecewise white noise model; assume the highest order term has discontinuous jumps between time steps

MatrixXd Q = L*w*L.transpose(); // process covariance, assume white noise

// ====================================================================================

int main(){
    KalmanFilter KF(x_0, P_0, F, Q);
    Tracker Tracker(initialPosition, initialVelocity);

    int N = 50; // number of time steps in the simulation

    std::vector<double> timeStep;

    std::vector<double> actual_X_Pos;
    std::vector<double> actual_Y_Pos;

    std::vector<double> measured_X_Pos;
    std::vector<double> measured_Y_Pos;

    std::vector<double> KF_X_Pos;
    std::vector<double> KF_Y_Pos;

    for(int i=1; i<=N; i++){
        timeStep.push_back(i);

        Tracker.updateMovements();
        Vector2d actualPosition = Tracker.getPosition();
        actual_X_Pos.push_back(actualPosition[0]);
        actual_Y_Pos.push_back(actualPosition[1]);

        Vector2d GPSReading = readGPS(actualPosition, GPS_X_STD, GPS_Y_STD);
        measured_X_Pos.push_back(GPSReading[0]);
        measured_Y_Pos.push_back(GPSReading[1]);

        KF.predict();
        KF.updateWithGPS(GPSReading, GPS_X_STD, GPS_Y_STD);
        KF_X_Pos.push_back(KF.getState()[0]);
        KF_Y_Pos.push_back(KF.getState()[2]);
    }
  
    using namespace matplot;

    // Uncomment to display X position plot
    plot(timeStep, measured_X_Pos, "ro", timeStep, KF_X_Pos, "b", timeStep, actual_X_Pos, "k--");
    title("X Position Plot");
    xlabel("Time Step (s)");
    ylabel("X Position (m)");
    ::matplot::legend({"GPS Measurements", "Linear KF", "Actual Position"});
    show();

    // // Uncomment to display Y position plot
    // plot(timeStep, measured_Y_Pos, "ro", timeStep, KF_Y_Pos, "b", timeStep, actual_Y_Pos, "k--");
    // title("Y Position Plot");
    // xlabel("Time Step (s)");
    // ylabel("Y Position (m)");
    // ::matplot::legend({"GPS Measurements", "Linear KF", "Actual Position"});
    // show();

    return 0;
}
