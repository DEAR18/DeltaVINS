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
    int idx;
    bool syncFlag;
    long long timestamp;

    bool operator<(long long t) { return timestamp < t; }

    bool operator>(long long t) { return timestamp > t; }

    bool operator==(long long t) { return timestamp == t; }

    bool operator<=(long long t) { return timestamp <= t; }

    bool operator<(int t) { return idx < t; }

    bool operator>(int t) { return idx > t; }

    bool operator==(int t) { return idx == t; }

    bool operator<=(int t) { return idx <= t; }
};

struct ImageData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    long long timestamp;

    cv::Mat image;
    int cam_id;
    cv::Mat right_image; // for stereo camera
    using Ptr = std::shared_ptr<ImageData>;
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