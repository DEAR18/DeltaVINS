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
    long long timestamp;
    int sensor_id;

    bool operator<(long long t) { return timestamp < t; }

    bool operator>(long long t) { return timestamp > t; }

    bool operator==(long long t) { return timestamp == t; }

    bool operator<=(long long t) { return timestamp <= t; }
};

struct ImageData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    long long timestamp;

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
    long long timestamp;
    int sensor_id;
    NavSatFixStatus status;
    double latitude;
    double longitude;
    double altitude;
    double covariance[9];
};

struct OdometerData {
    long long timestamp;

    int encoderL;
    int encoderR;
    float dEncoderL;
    float dEncoderR;

    float velocity;
    float angularVelocity;
};

}  // namespace DeltaVins