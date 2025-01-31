#pragma once
#include <string>
#include <vector>

namespace DeltaVins {
enum DataSrcType { DataSrcEuroc, DataSrcSynthetic,DataSrcROS2 };
enum class ROS2SensorType{
    MonoCamera,
    StereoCamera,
    IMU,
};
struct ROS2SensorTopic{
    ROS2SensorType type;
    std::vector<std::string> topics;
    int sensor_id;
    int queue_size;
};

struct Config {
    static void loadConfigFile(const std::string& configFile);

    static int DataSourceType;
    static std::string DataSourcePath;
    static int ImageStartIdx;
    static std::string CameraCalibFile;
    static float GyroBiasNoise2;
    static float AccBiasNoise2;
    static float GyroNoise2;
    static float AccNoise2;
    static float ImageNoise2;
    static int SerialRun;
    static int nImuSample;
    static int nImageSample;
    static int nImuPerImage;
    static int NoGUI;
    static int NoDebugOutput;
    static int NoResultOutput;
    static int DataFps;
    static std::string outputFileName;
    static std::string ResultOutputPath;
    static int CameraCalibration;
    static int RecordData;
    static int RunVIO;
    static int RecordIMU;
    static int RecordImage;
    static float ExposureTime;
    static float Gain;
    static int PlaneConstraint;
    static std::string DataSourceConfigFilePath;

    static void _clear();

    static std::string VisualizerServerIP;
    static int UploadImage;


    static std::vector<ROS2SensorTopic> ROS2SensorTopics;
};
}  // namespace DeltaVins
