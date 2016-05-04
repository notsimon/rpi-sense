#pragma once

#include <Eigen/Dense>

class KalmanFilter {
public:
    class StateVector : public Eigen::Matrix<float, 3, 1> {
    public:
        typedef Eigen::Matrix<float, 3, 1> Base;

        StateVector(void) : Base() {}

        // allows to construct from Eigen expressions
        template<typename OtherDerived>
        StateVector(const Eigen::MatrixBase<OtherDerived>& other) : Base(other) {}

        // allows to assign Eigen expressions
        template<typename OtherDerived>
        StateVector& operator= (const Eigen::MatrixBase <OtherDerived>& other) {
            this->Base::operator=(other);
            return *this;
        }

        // accessors
#define DEFINE_ACCESSORS(VAR, INDEX) \
        float VAR() const { \
            return (*this)(INDEX); \
        } \
        float VAR() { \
            return (*this)(INDEX); \
        }

        DEFINE_ACCESSORS(altitude,           0);
        DEFINE_ACCESSORS(pressure,           1);
        DEFINE_ACCESSORS(temperature,        2);
        //DEFINE_ACCESSORS(temperature_offset, 3);
#undef DEFINE_ACCESSORS
    };
    typedef Eigen::Matrix<float, StateVector::RowsAtCompileTime, StateVector::RowsAtCompileTime> StateCovMatrix;
    typedef Eigen::Matrix<float, 2, 1> MeasureVector;
    typedef Eigen::Matrix<float, MeasureVector::RowsAtCompileTime, MeasureVector::RowsAtCompileTime> MeasureCovMatrix;
    typedef Eigen::Matrix<float, MeasureVector::RowsAtCompileTime, StateVector::RowsAtCompileTime> MeasureFunMatrix;
    typedef Eigen::Matrix<float, StateVector::RowsAtCompileTime, MeasureVector::RowsAtCompileTime> KalmanGainMatrix;

    KalmanFilter();
    void operator()(float p, float t, float dt);
    const StateVector& state() const { return state_; }
    float& ref_pressure() { return ref_pressure_; }

private:
    float ref_pressure_ = 1008.30;
    StateVector state_;
    StateCovMatrix cov_;
};
