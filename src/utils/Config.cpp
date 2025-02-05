/**
 * This file is part of Delta_VIO.
 *
 * Delta_VIO is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Delta_VIO is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Delta_VIO. If not, see <http://www.gnu.org/licenses/>.
 */
#include "utils/Config.h"

#include <utils/utils.h>

#include <filesystem>

#include "precompile.h"

using namespace Eigen;
using namespace std;
using namespace cv;

namespace DeltaVins {
int Config::DataSourceType;
string Config::DataSourcePath;
int Config::SerialRun;
int Config::ImageStartIdx;

int Config::nImuSample;
int Config::nImageSample;
int Config::nImuPerImage;
float Config::ImageNoise2;
float Config::GyroNoise2;
float Config::AccNoise2;
float Config::GyroBiasNoise2;
float Config::AccBiasNoise2;
float Config::ExposureTime;
float Config::Gain;

string Config::CameraCalibFile;
string Config::outputFileName;
string Config::ResultOutputPath;

string Config::DataSourceConfigFilePath;
int Config::CameraCalibration;
int Config::NoDebugOutput;
int Config::NoResultOutput;
int Config::NoGUI;
int Config::MaxRunFPS;
int Config::RecordData;
int Config::RecordIMU;
int Config::RecordImage;
int Config::UploadImage;
int Config::RunVIO;
int Config::PlaneConstraint;
ResultOutputFormat Config::OutputFormat;
string Config::VisualizerServerIP;
int Config::MaxNumToTrack;
int Config::MaskSize;

std::vector<ROS2SensorTopic> Config::ROS2SensorTopics;

void Config::loadConfigFile(const std::string& configFile) {
    _clear();

    FileStorage config_file_cv, data_source_config_file_cv,
        camera_calib_file_cv;
    // get current path
    std::filesystem::path currentPath = std::filesystem::current_path();

    config_file_cv.open(configFile, FileStorage::READ);
    if (!config_file_cv.isOpened()) {
        throw std::runtime_error("fail to open config file at " + configFile);
    } else {
        LOGI("Load config :%s", configFile.c_str());
    }

    config_file_cv["DataSourceConfigFilePath"] >> DataSourceConfigFilePath;

    if (DataSourceConfigFilePath.empty()) {
        // get config file folder path
        std::filesystem::path config_path;
        config_path = std::filesystem::path(configFile).parent_path();
        DataSourceConfigFilePath = config_path.string() + "/DataSources.yaml";
    }

    data_source_config_file_cv.open(DataSourceConfigFilePath,
                                    FileStorage::READ);
    if (!data_source_config_file_cv.isOpened()) {
        throw std::runtime_error("fail to open data source config file at " +
                                 DataSourceConfigFilePath);
    } else {
        LOGI("Load data source config :%s", DataSourceConfigFilePath.c_str());
    }
    config_file_cv["CameraCalibrationPath"] >> CameraCalibFile;
    if (CameraCalibFile.empty()) {
        // get config file folder path
        std::filesystem::path config_path;
        config_path = std::filesystem::path(configFile).parent_path();
        CameraCalibFile = config_path.string() + "/calibrations.yaml";
    }
    camera_calib_file_cv.open(CameraCalibFile, FileStorage::READ);
    if (!camera_calib_file_cv.isOpened()) {
        throw std::runtime_error("fail to open camera calibration file at " +
                                 CameraCalibFile);
    } else {
        LOGI("Load camera calibration :%s", CameraCalibFile.c_str());
    }
    config_file_cv["PlaneConstraint"] >> PlaneConstraint;

    string temp;
    PlaneConstraint = false;
    data_source_config_file_cv["DataSourceType"] >> temp;
#ifndef USE_ROS2
    if (temp == "EUROC" || temp == "Euroc")
        DataSourceType = DataSrcEuroc;
    else if (temp == "Synthetic")
        DataSourceType = DataSrcSynthetic;
#else
    if (temp == "ROS2")
        DataSourceType = DataSrcROS2;
    else if (temp == "ROS2_bag") {
        DataSourceType = DataSrcROS2_bag;
    }
#endif
    else
        throw std::runtime_error("Unknown DataSource:" + temp);

    data_source_config_file_cv["DataSourcePath"] >> DataSourcePath;

    camera_calib_file_cv["ImuSampleFps"] >> nImuSample;
    camera_calib_file_cv["ImageSampleFps"] >> nImageSample;
    camera_calib_file_cv["PixelNoise"] >> ImageNoise2;
    camera_calib_file_cv["GyroNoise"] >> GyroNoise2;
    camera_calib_file_cv["AccNoise"] >> AccNoise2;
    camera_calib_file_cv["GyroBiasNoise"] >> GyroBiasNoise2;
    camera_calib_file_cv["AccBiasNoise"] >> AccBiasNoise2;
    nImuPerImage = nImuSample / nImageSample;
    if (nImuSample == 0 || nImageSample == 0 || nImuSample % nImageSample) {
        throw std::runtime_error(
            "Imu Sample Speed or Image Sample Speed Error, or Imu Sample Speed "
            "is not a multiple of Image Sample Speed.");
    }

    GyroNoise2 = GyroNoise2 * (GyroNoise2 * nImuSample);
    AccNoise2 = AccNoise2 * (AccNoise2 * nImuSample);
    GyroBiasNoise2 = GyroBiasNoise2 * (GyroBiasNoise2 * nImuSample);
    AccBiasNoise2 = AccBiasNoise2 * (AccBiasNoise2 * nImuSample);
    ImageNoise2 = ImageNoise2 * ImageNoise2;

    data_source_config_file_cv["ImageStartIdx"] >> ImageStartIdx;
    config_file_cv["SerialRun"] >> SerialRun;
    config_file_cv["NoGUI"] >> NoGUI;
    config_file_cv["NoDebugOutput"] >> NoDebugOutput;
    config_file_cv["MaxRunFPS"] >> MaxRunFPS;
    config_file_cv["RecordImu"] >> RecordIMU;
    config_file_cv["RecordImage"] >> RecordImage;
    config_file_cv["NoResultOutput"] >> NoResultOutput;
    config_file_cv["ExposureTime"] >> ExposureTime;
    config_file_cv["Gain"] >> Gain;
    config_file_cv["CameraCalibration"] >> CameraCalibration;
    config_file_cv["VisualizerServerIP"] >> VisualizerServerIP;
    config_file_cv["UploadImage"] >> UploadImage;
    config_file_cv["RunVIO"] >> RunVIO;
    config_file_cv["ResultOutputPath"] >> ResultOutputPath;
    config_file_cv["ResultOutputName"] >> outputFileName;
    config_file_cv["MaxNumToTrack"] >> MaxNumToTrack;
    config_file_cv["MaskSize"] >> MaskSize;

    if (RecordImage || RecordIMU) RecordData = 1;

    if (ResultOutputPath.empty()) {
        ResultOutputPath = "./";
    }

    existOrMkdir(ResultOutputPath + "/TestResults");
    if (outputFileName.empty()) {
        outputFileName = "outputPose";
    }
    outputFileName = ResultOutputPath + "/TestResults/" + outputFileName;

    if (DataSourceType == DataSrcROS2 || DataSourceType == DataSrcROS2_bag) {
        FileNode node = data_source_config_file_cv["ROSTopics"];
        for (FileNodeIterator it = node.begin(); it != node.end(); ++it) {
            ROS2SensorTopic topic;
            std::string topic_name;
            (*it)["TopicName"] >> topic_name;
            topic.topics.push_back(topic_name);
            std::string topic_type;
            (*it)["SensorType"] >> topic_type;
            if (topic_type == "StereoCamera") {
                topic.type = ROS2SensorType::StereoCamera;
            } else if (topic_type == "MonoCamera") {
                topic.type = ROS2SensorType::MonoCamera;
            } else if (topic_type == "IMU") {
                topic.type = ROS2SensorType::IMU;
            } else {
                throw std::runtime_error("Unknown ROS2 topic type: " +
                                         topic_type);
            }
            (*it)["SensorID"] >> topic.sensor_id;

            if (topic.type == ROS2SensorType::StereoCamera) {
                (*it)["RightTopicName"] >> topic_name;
                topic.topics.push_back(topic_name);
            }
            (*it)["TopicQueueSize"] >> topic.queue_size;
            ROS2SensorTopics.push_back(topic);
        }
    }

    config_file_cv["ResultOutputFormat"] >> temp;
    if (temp == "TUM") {
        OutputFormat = ResultOutputFormat::TUM;
        outputFileName += ".tum";
    } else if (temp == "KITTI") {
        OutputFormat = ResultOutputFormat::KITTI;
        outputFileName += ".kitti";
    } else if (temp == "EUROC") {
        OutputFormat = ResultOutputFormat::EUROC;
        outputFileName += ".euroc";
    } else {
        throw std::runtime_error("Unknown ResultOutputFormat: " + temp);
    }

    if(DataSourceType == DataSrcROS2_bag){
        SerialRun = 1; // run in serial mode if data source is ROS2_bag
    }

}

void Config::_clear() {
    RecordData = 0;
    RecordIMU = 0;
    RecordImage = 0;
    nImuSample = 0;
    nImageSample = 0;
    nImuPerImage = 0;
    NoResultOutput = 0;
    ExposureTime = 0.01f;
    Gain = 1.0f;
    CameraCalibration = 0;
    UploadImage = 0;
    RunVIO = 0;
    PlaneConstraint = 0;
}

#if 0
    void Config::generateTemplateFile()
	{
		cv::FileStorage configFile("Config/Config.yaml",FileStorage::WRITE);
		configFile.write("PixelNoise",2.);
		configFile.write("GyroNoise",2.);
		configFile.write("AccNoise",2.);
		configFile.write("GyroBiasNoise",2.);
		configFile.write("AccBiasNoise",2.);
		configFile.write("ImageStartIdx",2);
		configFile.write("SerialRun",1);
		configFile.write("CameraCalibrationPath","path");
		configFile.write("ImuSampleFps","400");
		configFile.write("DatasetPath","Path");
		configFile.write("DatasetType","Type");

	}
#endif

}  // namespace DeltaVins
