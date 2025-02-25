//
// Created by chenguojun on 2020/6/10.
//

#pragma once
#include <utils/basicTypes.h>

namespace DeltaVins {

struct ImuData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vector3f gyro;
    Vector3f acc;
    int64_t timestamp;
    int sensor_id;

    bool operator<(long long t) { return timestamp < t; }

    bool operator>(long long t) { return timestamp > t; }

    bool operator==(long long t) { return timestamp == t; }

    bool operator<=(long long t) { return timestamp <= t; }
};

struct ImageData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    int64_t timestamp;

    cv::Mat image;
    int sensor_id;
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
    int64_t timestamp;
    int sensor_id;
    NavSatFixStatus status;
    double latitude;
    double longitude;
    double altitude;
    double covariance[9];
};

struct OdometerData {
    int64_t timestamp;

    int encoderL;
    int encoderR;
    float dEncoderL;
    float dEncoderR;

    float velocity;
    float angularVelocity;
};

}  // namespace DeltaVins