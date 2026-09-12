#include "Linear_Kalman_Filter/pseudosensors.h"

// Tracker constructor implementation
Tracker::Tracker(Vector2d initialPosition, Vector2d initialVelocity, double noiseStd){
    this->position = initialPosition;
    this->velocity = initialVelocity;
    this->velocitySetpoint = initialVelocity;
    this->noiseStd = noiseStd;
}

void Tracker::updateMovements(){
    std::random_device rd{};
    std::mt19937 generator{ rd() }; // generate pseudorandom numbers

    std::normal_distribution<double> distribution(0, noiseStd);

    // Update noisy movement attributes for the current time step
    double noise = distribution(generator);
    velocity[0] = velocitySetpoint[0] + noise;
    velocity[1] = velocitySetpoint[1] + noise;
    position[0] = position[0] + velocity[0];
    position[1] = position[1] + velocity[1];
}

// Pseudor sensor function implementations
Vector2d readGPS(Vector2d actualPosition, double xStd, double yStd){
    std::random_device rd{};
    std::mt19937 generator{ rd() }; // generate pseudorandom numbers

    std::normal_distribution<double> distributionX(0, xStd);
    std::normal_distribution<double> distributionY(0, yStd);

    Vector2d noisyReading;
    noisyReading[0] = actualPosition[0] + distributionX(generator);
    noisyReading[1] = actualPosition[1] + distributionY(generator);

    return noisyReading;
}

