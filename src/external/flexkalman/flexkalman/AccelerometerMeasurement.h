// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PS Move tracker code that is expensive to compile.
 *
 * Typically built as a part of t_kalman.cpp to reduce incremental build times.
 *
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 * @author Pete Black <pblack@collabora.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup aux_tracking
 */

#include "BaseTypes.h"
#include "flexkalman/FlexibleKalmanBase.h"

namespace flexkalman {
class AccelerometerMeasurement
    : public flexkalman::MeasurementBase<AccelerometerMeasurement> {

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static constexpr size_t Dimension = 3;
    using MeasurementVector = types::Vector<Dimension>;
    using MeasurementSquareMatrix = types::SquareMatrix<Dimension>;

    AccelerometerMeasurement(types::Vector<3> const &accel,
                             types::Vector<3> const &gravity_ref,
                             types::Vector<3> const &variance)
        : accel_(accel), gravity_ref_(gravity_ref), covariance_(variance.asDiagonal()) {}

    template<typename State>
    MeasurementSquareMatrix const &getCovariance(State const & /*s*/)

    {
        return covariance_;
    }

    template<typename State>
    MeasurementVector predictMeasurement(State const &s) const {
        // TODO: Find a way to separate gravity from acceleration
        auto q = s.a().getCombinedQuaternion();
        //return gravity_ref_ + q * (s.a().acceleration() - s.b().accelBias());
        return q * (s.a().acceleration() - s.b().accelBias());
    }

    template<typename State>
    MeasurementVector getResidual(MeasurementVector const &predictedMeasurement,
                                  State const &s) const {
        return accel_ - predictedMeasurement;
    }

    template<typename State>
    MeasurementVector getResidual(State const &s) const {
        return getResidual(predictMeasurement(s), s);
    }

  private:
    MeasurementVector accel_;
    MeasurementVector gravity_ref_;
    MeasurementSquareMatrix covariance_;
};
} // namespace flexkalman
