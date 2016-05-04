#include "kalman.h"
#include <iostream>
#include <cassert>

static const float g = 9.80665; // gravitational acceleration
static const float m = 0.028965338; // molar mass of air
static const float r = 8.3144621; // gas constant of air

KalmanFilter::KalmanFilter() {
    using std::pow;

    state_ << 10, ref_pressure_, 10;
    cov_.setIdentity();
}

using StateCovMatrix = KalmanFilter::StateCovMatrix;
using StateVector = KalmanFilter::StateVector;

inline StateCovMatrix linear_forward(const float h, /*< height */
                                     const float t, /*< ambiant temperature */
                                     const float p0, /*< pressure at reference point */
                                     const float dt /*< time step */) {
    using std::exp;
    using std::pow;

    StateCovMatrix out;
    out << exp(dt), 0, 0,

           -g*m*p0*exp(dt)*exp(-g*h*m/(r*t))/(r*t) + g*m*p0*exp(-g*h*m/(r*t))/(r*t),
           1,
           g*h*m*p0*exp(dt)*exp(-g*h*m/(r*t))/(r*pow(t, 2)) - g*h*m*p0*exp(-g*h*m/(r*t))/(r*pow(t, 2)),

           0, 0, exp(dt);

    return out;
}

inline StateVector forward(StateVector& state, const float p0) {
    using std::exp;
    const float h = state.altitude();
    const float t = state.temperature();
    StateVector out;
    out << state.altitude(),
           p0 * exp(-g*m*h/(r*t)),
           state.temperature();
    return out;
}

void KalmanFilter::operator()(const float p, const float t, const float dt) {
    std::cout << "state = " << state_.transpose() << std::endl;
    std::cout << "covar = " << std::endl << cov_ << std::endl;

    // -------------
    // -- predict --
    // -------------

    StateCovMatrix process_noise;
    process_noise.setZero();
    process_noise.diagonal()[0]= pow( 0.1, 2); // altitude
    process_noise.diagonal()[1]= pow( 0.2, 2); // pressure
    process_noise.diagonal()[2]= pow(1.0 * dt, 2); // room temperature
    //process_noise_.diagonal()[3]= pow(   1, 2); // temperature offset

    const StateCovMatrix F = linear_forward(state_.altitude(),
                                            state_.temperature(),
                                            ref_pressure_,
                                            dt);

    const StateVector predict = forward(state_, ref_pressure_);
    const StateCovMatrix predict_cov = F*cov_*F.transpose() + process_noise;

    // ------------
    // -- update --
    // ------------

    MeasureCovMatrix measure_noise;
    measure_noise.setZero();
    measure_noise.diagonal()[0] = 0.0003;
    measure_noise.diagonal()[1] = 0.0002;

    const float alpha = 0;
    MeasureVector measure;
    measure << p, t;

    MeasureFunMatrix H;
    H << 0, 1, alpha,
         0, 0,     1;

    MeasureVector residual = measure - H*predict;
    KalmanGainMatrix gain = predict_cov * H.transpose() * (H*predict_cov*H.transpose() + measure_noise).inverse();
    state_ = predict + gain*residual;
    cov_ = (StateCovMatrix::Identity() - gain*H)*predict_cov;
}
