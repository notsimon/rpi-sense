#include "kalman.h"
#include <iostream>
#include <cassert>

static const float g = 9.80665; // gravitational acceleration
static const float m = 0.028965338; // molar mass of the gas
static const float r = 8.3144621; // gas constant

KalmanFilter::KalmanFilter() {
    process_noise_.setZero();
    process_noise_.diagonal().fill(1.0);

    measure_noise_.setZero();
    measure_noise_.diagonal().fill(1.0);

    state_ << 10, ref_pressure_, 10, 0;
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
    out << exp(dt), 0, 0, 0,

           -g*m*p0*exp(dt)*exp(-g*m*h/(r*t))/(r*t) + g*m*p0*exp(-g*m*h/(r*t))/(r*t),
           1,
           g*m*h*p0*exp(dt)*exp(-g*m*h/(r*t))/(r*pow(t, 2)) - g*m*h*p0*exp(-g*m*h/(r*t))/(r*pow(t, 2)),
           0,

           0, 0, exp(dt), 0,
           0, 0, 0, exp(dt);

    return std::move(out);
}

inline StateVector forward(StateVector& state, const float p0) {
    using std::exp;
    const float h = state.altitude();
    const float t = state.temperature();
    StateVector out;
    out << state.altitude(),
           p0 * exp(-g*m*h/(r*t)),
           state.temperature(),
           state.temperature_offset();
    return out;
}

void KalmanFilter::operator()(const float p, const float t, const float dt) {
    std::cout << "state = " << std::endl << state_ << std::endl;
    std::cout << "covar = " << std::endl << cov_ << std::endl;

    // predict
    const StateCovMatrix F = linear_forward(state_.altitude(),
                                            state_.temperature(),
                                            ref_pressure_,
                                            dt);

    const StateVector predict = forward(state_, ref_pressure_);
    const StateCovMatrix predict_cov = F*cov_*F.transpose() + process_noise_;

    // update
    const float alpha = 0;
    MeasureVector measure;
    measure << p, t;

    MeasureFunMatrix H;
    H << 0, 1, alpha, alpha,
         0, 0,     1,     1;

    MeasureVector residual = measure - H*predict;
    KalmanGainMatrix gain = predict_cov * H.transpose() * (H*predict_cov*H.transpose() + measure_noise_).inverse();
    state_ = predict + gain*residual;
    cov_ = (StateCovMatrix::Identity() - gain*H)*predict_cov;
}
