#include "utils/SensorConfig.h"

#include <filesystem>
#include <opencv2/opencv.hpp>

#include "utils/log.h"

namespace DeltaVins {

void SensorConfig::LoadConfig(const std::string& config_path) {
    if (!std::filesystem::exists(config_path)) {
        LOGW("Config file not found: %s", config_path.c_str());
        return;
    }
    // list all yaml files in the config_path
    std::vector<std::string> yaml_files;
    for (const auto& entry : std::filesystem::directory_iterator(config_path)) {
        if (entry.path().extension() == ".yaml") {
            yaml_files.push_back(entry.path().string());
        }
    }

    for (const auto& yaml_file : yaml_files) {
        LOGI("Loading config file: %s", yaml_file.c_str());
        cv::FileStorage fs(yaml_file, cv::FileStorage::READ);
        std::string sensor_type;
        fs["SensorType"] >> sensor_type;
        int sensor_id;
        fs["SensorId"] >> sensor_id;
        if (sensor_type == "IMU") {
            IMUParams imu_params;
            fs["ImuSampleFps"] >> imu_params.fps;
            fs["GyroNoise"] >> imu_params.gyro_noise;
            fs["AccNoise"] >> imu_params.acc_noise;
            fs["GyroBiasNoise"] >> imu_params.gyro_bias_noise;
            fs["AccBiasNoise"] >> imu_params.acc_bias_noise;
            imu_params.sensor_id = sensor_id;
            imu_params_[sensor_id] = imu_params;
            cv::Mat Tbs;
            fs["Tbs"] >> Tbs;
            Eigen::Matrix4f Tbs_eigen;
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    Tbs_eigen(i, j) = Tbs.at<double>(i, j);
                }
            }
            Eigen::Isometry3f Tbs_eigen_iso;
            Tbs_eigen_iso.linear() = Tbs_eigen.block<3, 3>(0, 0);
            Tbs_eigen_iso.translation() = Tbs_eigen.block<3, 1>(0, 3);
            Transform<float> Tbi;
            Tbi.timestamp = 0;
            Tbi.frame_id = "body";
            Tbi.child_frame_id = "imu" + std::to_string(sensor_id);
            Tbi.T_parent_child = Tbs_eigen_iso;
            Tfs<float>::Instance().AddStaticTransform(Tbi);
        } else if (sensor_type == "StereoCamera" ||
                   sensor_type == "MonoCamera") {
            CameraParams camera_params;
            camera_params.sensor_id = sensor_id;
            fs["ImageSampleFps"] >> camera_params.fps;
            fs["PixelNoise"] >> camera_params.image_noise;
            bool is_stereo;
            fs["IsStereo"] >> is_stereo;
            camera_params_[sensor_id] = camera_params;

            CamModel::Ptr cam_model = CamModel::CreateFromConfig(fs);
            cam_models_[sensor_id] = cam_model;
            cv::Mat Tbs;
            fs["Tbs"] >> Tbs;
            Eigen::Matrix4f Tbs_eigen;
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    Tbs_eigen(i, j) = Tbs.at<double>(i, j);
                }
            }
            Eigen::Isometry3f Tbs_eigen_iso;
            Tbs_eigen_iso.linear() = Tbs_eigen.block<3, 3>(0, 0);
            Tbs_eigen_iso.translation() = Tbs_eigen.block<3, 1>(0, 3);
            Transform<float> Tbi;
            Tbi.timestamp = 0;
            Tbi.frame_id = "body";
            Tbi.child_frame_id = "camera" + std::to_string(sensor_id);
            Tbi.T_parent_child = Tbs_eigen_iso;
            Tfs<float>::Instance().AddStaticTransform(Tbi);

            if (sensor_type == "StereoCamera" && is_stereo) {
                fs["Tbs_right"] >> Tbs;
                for (int i = 0; i < 4; ++i) {
                    for (int j = 0; j < 4; ++j) {
                        Tbs_eigen(i, j) = Tbs.at<double>(i, j);
                    }
                }
                Eigen::Isometry3f Tbs_eigen_iso_right;
                Tbs_eigen_iso_right.linear() = Tbs_eigen.block<3, 3>(0, 0);
                Tbs_eigen_iso_right.translation() = Tbs_eigen.block<3, 1>(0, 3);
                Transform<float> Tbi_right;
                Tbi_right.timestamp = 0;
                Tbi_right.frame_id = "body";
                Tbi_right.child_frame_id =
                    "camera" + std::to_string(sensor_id) + "_right";
                Tbi_right.T_parent_child = Tbs_eigen_iso_right;
                Tfs<float>::Instance().AddStaticTransform(Tbi_right);
            }
            camera_params_[sensor_id] = camera_params;
        }
    }

    // Todo: Here we only set the extrinsic of the camera0 to the imu0
    for (const auto& [sensor_id, cam_model] : cam_models_) {
        if (sensor_id == 0) {
            Transform<float> Tci_cam0_imu0;
            if (!Tfs<float>::Instance().GetTransform("camera0", "imu0",
                                                     Tci_cam0_imu0)) {
                throw std::runtime_error(
                    "No transform found for camera0 to imu0");
            }
            cam_model->SetVIExtrinsic(Tci_cam0_imu0.Rotation(),
                                      Tci_cam0_imu0.Translation(), 0);
            if (cam_model->IsStereo()) {
                Transform<float> Tci_cam0_imu0_right;
                if (!Tfs<float>::Instance().GetTransform(
                        "camera0_right", "imu0", Tci_cam0_imu0_right)) {
                    throw std::runtime_error(
                        "No transform found for camera0_right to imu0");
                }
                cam_model->SetVIExtrinsic(Tci_cam0_imu0_right.Rotation(),
                                          Tci_cam0_imu0_right.Translation(), 1);
            }
        }
    }
}
}  // namespace DeltaVins