#pragma once

#include <string>
#include <vector>

#include "Algorithm/vision/camModel/camModel.h"
#include "utils/tf.h"

namespace DeltaVins {

struct IMUParams {
    int sensor_id;
    int fps;
    int acc_fps;   // for separate acc and gyro message
    int gyro_fps;  // for separate acc and gyro message
    double gyro_noise;
    double acc_noise;
    double gyro_bias_noise;
    double acc_bias_noise;
};

struct CameraParams {
    int sensor_id;
    int fps;
    double image_noise;
};

class SensorConfig {
   public:
    static SensorConfig& Instance() {
        static SensorConfig instance;
        return instance;
    }

    CamModel::Ptr GetCamModel(int sensor_id) { return cam_models_[sensor_id]; }

    IMUParams GetIMUParams(int sensor_id) { return imu_params_[sensor_id]; }

    CameraParams GetCameraParams(int sensor_id) {
        return camera_params_[sensor_id];
    }

    bool LoadConfig(const std::string& config_path);

   private:
    SensorConfig() {}

    SensorConfig(const SensorConfig&) = delete;
    SensorConfig& operator=(const SensorConfig&) = delete;

    std::unordered_map<int, IMUParams> imu_params_;
    std::unordered_map<int, CamModel::Ptr> cam_models_;

    std::unordered_map<int, CameraParams> camera_params_;
};

}  // namespace DeltaVins