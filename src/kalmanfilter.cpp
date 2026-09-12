#include "Linear_Kalman_Filter/kalmanfilter.h"

// ================ KALMAN FILTER CLASS FUNCTION IMPLEMENTATIONs ================
KalmanFilter::KalmanFilter(VectorXd x, MatrixXd P, MatrixXd F, MatrixXd Q){
    this->x = x;
    this->P = P;
    this->F = F;
    this->Q = Q;
    this->B = MatrixXd::Zero(F.rows(), F.cols()); 
    this->u = MatrixXd::Zero(x.rows(), x.cols());  
}; 

void KalmanFilter::predict(){
    VectorXd x = getState();
    MatrixXd P = getCovariance();

    VectorXd x_pred = F*x + B*u;
    MatrixXd P_pred = F*P*F.transpose() + Q;

    setState(x_pred);
    setCovariance(P_pred);
}

void KalmanFilter::updateWithGPS(Vector2d GPSReading, double GPSXStd, double GPSYStd){
    VectorXd x_hat = getState();
    MatrixXd P_hat = getCovariance();

    Vector2d z = GPSReading; // measurement mean
    
    MatrixXd H {{1, 0, 0, 0},
                {0, 0, 1, 0}
    }; // measurement matrix to convert state variables into GPS position readings in m

    MatrixXd R{{GPSXStd*GPSXStd, 0},
               {0, GPSYStd*GPSYStd},
    }; // measurement noise covariance

    VectorXd y = z - H*x_hat; // compute residual i.e. difference between sensor measurement and the model's predicted value
    MatrixXd K = P_hat*H.transpose() * (H*P_hat*H.transpose() + R).inverse(); // compute Kalman gain, the weighting factor between measurement and model prediction
    VectorXd x = x_hat + K*y;  // compute new state vector
    MatrixXd P = (MatrixXd::Identity(P_hat.rows(),P_hat.cols()) - K*H)*P_hat; // compute new covariance matrix

    setState(x);
    setCovariance(P);
}