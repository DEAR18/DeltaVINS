//
// Created by chenguojun on 2020/6/10.
//

#pragma once
#include "utils/basicTypes.h"

namespace DeltaVins {

struct ImuData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vector3f gyro{Eigen::Vector3f::Zero()};
    Vector3f acc{Eigen::Vector3f::Zero()};
    int64_t timestamp{0};
    int sensor_id{0};

    bool operator<(int64_t t) { return timestamp < t; }

    bool operator>(int64_t t) { return timestamp > t; }

    bool operator==(int64_t t) { return timestamp == t; }

    bool operator<=(int64_t t) { return timestamp <= t; }
};

struct ImageData {
    int64_t timestamp{0};

    cv::Mat image;
    int sensor_id{0};
    cv::Mat right_image;  // for stereo camera
    using Ptr = std::shared_ptr<ImageData>;
};

enum class NavSatFixStatus {
    NO_FIX = -1,
    FIXED = 0,
    SBAS_FIX = 1,
    GBAS_FIX = 2,
};

struct NavSatFixData {
    int64_t timestamp{0};
    int sensor_id{0};
    NavSatFixStatus status{NavSatFixStatus::NO_FIX};
    double latitude{0.0};
    double longitude{0.0};
    double altitude{0.0};
    double covariance[9]{0.0};
};

struct OdometerData {
    int64_t timestamp{0};
    int sensor_id{0};
    int encoderL{0};
    int encoderR{0};
    float dEncoderL{0.f};
    float dEncoderR{0.f};

    float velocity{0.f};
    float angularVelocity{0.f};

    bool operator<(int64_t t) { return timestamp < t; }
    bool operator>(int64_t t) { return timestamp > t; }
    bool operator==(int64_t t) { return timestamp == t; }
    bool operator<=(int64_t t) { return timestamp <= t; }
    bool operator>=(int64_t t) { return timestamp >= t; }
};

}  // namespace DeltaVins