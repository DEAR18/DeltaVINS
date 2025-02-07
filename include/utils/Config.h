#pragma once
#include <string>
#include <vector>

namespace DeltaVins {
enum DataSrcType {
    DataSrcEuroc,
    DataSrcSynthetic,
    DataSrcROS2,
    DataSrcROS2_bag
};
enum class ROS2SensorType { MonoCamera, StereoCamera, IMU, GNSS };
struct ROS2SensorTopic {
    ROS2SensorType type;
    std::vector<std::string> topics;
    int sensor_id;
    int queue_size;
};

enum class ResultOutputFormat {
    TUM,
    KITTI,
    EUROC,
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
    static int MaxRunFPS;
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
    static ResultOutputFormat OutputFormat;
    static void _clear();
    static int MaxNumToTrack;
    static int MaskSize;

    static std::string VisualizerServerIP;
    static int UploadImage;

    static std::vector<ROS2SensorTopic> ROS2SensorTopics;
};
}  // namespace DeltaVins
