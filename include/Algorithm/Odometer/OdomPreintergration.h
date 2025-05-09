#pragma once
#include <sophus/so3.hpp>

#include "utils/typedefs.h"
#include "utils/tf.h"
#include "utils/utils.h"

namespace DeltaVins {
struct OdomPreintergration {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*
    MatrixXf CalObservationJacobian(const Matrix3f& R0, const Vector3f& p0,
                                    const Matrix3f& R1,
                                    const Vector3f& p1) const {
        Transform<float> Tbi;
        Tfs<float>::Instance().GetTransform("body", "imu0", Tbi);
        Transform<float> Tib = Tbi.Inverse();

        MatrixXf H = MatrixXf::Zero(
            6, 12);  // order: dobs_dR0, dobs_dp0, dobs_dR1, dobs_dp1
        H.block<3, 3>(0, 0) = -Tbi.Rotation() * R1.transpose() * R0;
        H.block<3, 3>(0, 6) = Tbi.Rotation();
        H.block<3, 3>(3, 0) =
            crossMat<float>(Tbi.Rotation() * R0.transpose() *
                            (R1 * Tib.Translation() + p1 - p0)) *
            Tbi.Rotation();
        H.block<3, 3>(3, 3) = -Tbi.Rotation() * R0.transpose();
        H.block<3, 3>(3, 6) = -Tbi.Rotation() * R0.transpose() * R1 *
                              crossMat<float>(Tib.Translation());
        H.block<3, 3>(3, 9) = Tbi.Rotation() * R0.transpose();

        // output jacobian of 2D observation [obs_theta, obs_x, obs_y]
        MatrixXf output_H = MatrixXf::Zero(3, 12);
        output_H.row(0) = H.row(2);
        output_H.row(1) = H.row(3);
        output_H.row(2) = H.row(4);
        return output_H;
    }

    VectorXf CalObservationResidual(const Matrix3f& R0, const Vector3f& p0,
                                    const Matrix3f& R1,
                                    const Vector3f& p1) const {
        Transform<float> Tbi;
        Tfs<float>::Instance().GetTransform("body", "imu0", Tbi);
        Transform<float> Tib = Tbi.Inverse();

        VectorXf residual = VectorXf::Zero(6);
        Matrix3f predicted_R =
            Tbi.Rotation() * R0.transpose() * R1 * Tib.Rotation();
        Matrix3f observed_R =
            Eigen::AngleAxisf(dtheta, Vector3f(0.f, 0.f, 1.f)).matrix();
        residual.head<3>() =
            Sophus::SO3f(observed_R * predicted_R.transpose()).log();
        Vector3f predicted_P = Tbi.Rotation() * R0.transpose() *
                                   (R1 * Tib.Translation() + p1 - p0) -
                               Tbi.Rotation() * Tib.Translation();
        Vector3f observed_P = Vector3f(dx, dy, 0.f);
        residual.tail<3>() = observed_P - predicted_P;

        // output residual of 2D observation [obs_theta, obs_x, obs_y]
        VectorXf output_residual = VectorXf::Zero(3);
        output_residual(0) = residual(2);
        output_residual(1) = residual(3);
        output_residual(2) = residual(4);
        return output_residual;
    }
    */

    int sensor_id{0};
    int64_t t0{0};  // first data timestamp
    int64_t t1{0};  // last data timestamp
    int64_t dT{0};  // delta time

    // 2D observation; coordinate frame: forward, left, upward
    float dtheta{0.f};               // delta theta
    float dx{0.f};                   // delta position x
    float dy{0.f};                   // delta position y
    Matrix3f cov{Matrix3f::Zero()};  // covariance matrix, order: theta,x,y
};
}  // namespace DeltaVins
