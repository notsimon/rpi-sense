#include "kalman.h"
#include <iostream>
#include <iomanip>
#include <cassert>

static const float g = 9.80665; // gravitational acceleration
static const float m = 0.028965338; // molar mass of air
static const float r = 8.3144621; // gas constant of air

KalmanFilter::KalmanFilter() {
    state_ << 0, ref_pressure_, 15 + 273.15, 0;

    cov_.setZero();
    cov_.setIdentity();
}

using StateCovMatrix = KalmanFilter::StateCovMatrix;
using StateVector = KalmanFilter::StateVector;

inline StateCovMatrix state_transition_matrix(const StateVector& state,
                                              const float p0, /*< pressure at reference point */
                                              const float dt /*< time step */) {
    using std::exp;
    using std::pow;

    const float h = state.altitude();
    const float t = state.temperature();

    StateCovMatrix out;
    out << 1, 0, 0, 0,
           -g*m*p0*exp(-g*h*m/(t*r))/(t*r),
           0,
           g*h*m*p0*exp(-g*h*m/(t*r))/(pow(t, 2)*r),
           0,
           0, 0, 1, dt,
           0, 0, 0, 1;

    return out;
}

inline StateVector forward(StateVector& state,
                           const float p0,
                           const float dt) {
    using std::exp;
    const float h = state.altitude();
    const float t = state.temperature();
    StateVector out;
    out << state.altitude(),
           p0 * exp(-g*m*h/(r*t)),
           state.temperature() + state.temperature_deriv()*dt,
           state.temperature_deriv();
    return out;
}

void KalmanFilter::operator()(const float p, const float t, const float dt) {
    std::cout << "state = " << std::fixed << std::setprecision(2) << state_.transpose() << std::endl;
    std::cout << "covar = " << std::endl << cov_ << std::endl;

    // -------------
    // -- predict --
    // -------------

    StateCovMatrix process_noise;
    process_noise.setZero();
    process_noise.diagonal()[0] = 0;//pow(0.0001, 2); // altitude
    process_noise.diagonal()[1] = 0;//pow(0.0002, 2); // pressure
    process_noise.diagonal()[2] = pow(1.0 * dt / 3600, 2); // room temperature
    process_noise.diagonal()[3] = pow(1.0, 2); // temperature derivative

    const StateCovMatrix A = state_transition_matrix(state_, ref_pressure_, dt);
    const StateCovMatrix F = StateCovMatrix::Identity() + A*dt;

    std::cout << "A = " << std::endl << A << std::endl;

    const StateVector predict = forward(state_, ref_pressure_, dt);
    const StateCovMatrix predict_cov = F*cov_*F.transpose() + process_noise;

    // ------------
    // -- update --
    // ------------

    MeasureCovMatrix measure_noise;
    measure_noise.setZero();
    measure_noise.diagonal()[0] = 0.1;
    measure_noise.diagonal()[1] = 0.01;

    const float alpha = 0;
    MeasureVector measure;
    measure << p, t;

    MeasureFunMatrix H;
    H << 0, 1, alpha, 0,
         0, 0,     1, 0;

    MeasureVector residual = measure - H*predict;
    KalmanGainMatrix gain = predict_cov * H.transpose() * (H*predict_cov*H.transpose() + measure_noise).inverse();
    state_ = predict + gain*residual;
    cov_ = (StateCovMatrix::Identity() - gain*H)*predict_cov;
}
