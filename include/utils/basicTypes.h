//
// Created by chenguojun on 2020/6/10.
//

#pragma once


namespace DeltaVins{



    constexpr auto EigenMajorType = Eigen::ColMajor;

    using Vector2f = Eigen::Matrix<float, 2, 1, EigenMajorType>;

    using Vector3f = Eigen::Matrix<float, 3, 1, EigenMajorType>;
    using Matrix2f = Eigen::Matrix<float, 2, 2, EigenMajorType>;
    using Matrix23f = Eigen::Matrix<float, 2, 3, EigenMajorType>;
    using Matrix3f = Eigen::Matrix<float, 3, 3, EigenMajorType>;
    using Matrix9f = Eigen::Matrix<float, 9, 9, EigenMajorType>;


    using Quaternionf =  Eigen::Quaternion<float>;

    using Vector2d = Eigen::Matrix<double, 2, 1, EigenMajorType>;

    using Vector3d = Eigen::Matrix<double, 3, 1, EigenMajorType>;
    using Matrix2d = Eigen::Matrix<double, 2, 2, EigenMajorType>;
    using Matrix23d = Eigen::Matrix<double, 2, 3, EigenMajorType>;
    using Matrix3d = Eigen::Matrix<double, 3, 3, EigenMajorType>;

    using MatrixXd = Eigen::Matrix<double, -1, -1, EigenMajorType>;
    using VectorXd = Eigen::Matrix<double, -1, 1, EigenMajorType>;

    using Quaterniond = Eigen::Quaternion<double>;

    using MatrixXf = Eigen::Matrix<float, -1, -1, EigenMajorType>;
    using VectorXf = Eigen::Matrix<float, -1, 1, EigenMajorType>;
}