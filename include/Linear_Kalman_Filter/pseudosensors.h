#pragma once

#include <Eigen/Dense>
#include <random>
#include <cmath>

using Eigen::Vector2d;

/**
 * @class Tracker
 * @brief Simulates the 2D movement of a constant velocity object affected by noise 
**/
class Tracker{
    private:
        Vector2d position;
        Vector2d velocity;
        Vector2d velocitySetpoint;
        double noiseStd; // noise

    public:
        /** 
         * @brief Constructor for Tracker object
         * 
         * @param initialPosition 2D vector for the initial position in m
         * @param initialVelocity 2D vector for the initial velocity in m/s
         * @param noiseStd Standard deviation for white noise with mean 0
         * 
         * @return None
        **/
        Tracker(Vector2d initialPosition, Vector2d initialVelocity, double noiseStd=1);

        /**
         * @brief Simulates an object's noisy movements in one time step
         * 
         * @return None
        **/
        void updateMovements();

        Vector2d getPosition() const {return position;}
        Vector2d getVelocity() const {return velocity;}
};

/** 
 * @brief Simulates noisy 2D GPS sensor readings
 * 
 * @param actualPosition 2D vector for actual position in m
 * @param xStd Standard deviation for X position measurement in m
 * @param yStd Standard deviation for Y position measurement in m
 * 
 * @return 2D vector for the current position reading, including noise effects
**/
Vector2d readGPS(Vector2d actualPosition, double xStd=1, double yStd=1);

