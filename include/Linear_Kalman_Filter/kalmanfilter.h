#pragma once

#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector2d;

/** 
 * @class KalmanFilter
 * @brief OOP implementation of a linear Kalman Filter
 * 
**/ 
class KalmanFilter{
    private:
        VectorXd x; // state mean
        MatrixXd P; // state covariance
        MatrixXd F; // state transition matrix
        MatrixXd Q; // process covariance, assume white noise
        MatrixXd B; // control input matrix
        MatrixXd u; // control inputs to system
    
    public:
        /** 
         * @brief Constructor for linear Kalman Filter initialisation,
         * initialises B and u as zero matrices/vectors
        **/
        KalmanFilter(VectorXd x, MatrixXd P, MatrixXd F, MatrixXd Q);

        void setState(VectorXd& state){x = state;}
        void setCovariance(MatrixXd& covariance){P = covariance;}
        void setControlMatrix(MatrixXd& controlMatrix){B = controlMatrix;}
        void setControlInputs(MatrixXd& controlInputs){u = controlInputs;}

        VectorXd getState() const {return x;}
        MatrixXd getCovariance() const {return P;}
        MatrixXd getControlMatrix() const {return B;}
        MatrixXd getControlInputs() const {return u;}

        /** 
         * @brief Prediction step of linear Kaman Fiter. Uses the process model to 
         * predict system state at the next time step and adjusts belief to account for 
         * prediction uncertainty.  
        **/
        void predict(); 

        /** 
         * @brief Updates state mean and covariance using GPS position measurements
         * 
         * @param GPSReading 2D vector (x, y) GPS position reading in m
         * @param GPSXStd Standard deviation for GPS x position measurement
         * @param GPSYStd Standard deviation for GPS y position measurement
         * 
         * @return None
        **/
        void updateWithGPS(Vector2d GPSReading, double GPSXStd, double GPSYStd);
};