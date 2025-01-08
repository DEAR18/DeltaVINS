#pragma once

namespace DeltaVins {
enum DataSrcType { DataSrcEuroc, DataSrcSynthetic };

struct Config {
    static void loadConfigFile(const std::string& datasetDir,
                               const std::string& outputName);

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
    static std::string testName;
    static int NoDebugOutput;
    static int NoResultOutput;
    static int DataFps;
    static std::string outputFileName;
    static int CameraCalibration;
    static cv::FileStorage m_configFile;
    static int RecordData;
    static int RunVIO;
    static int RecordIMU;
    static int RecordImage;
    static float ExposureTime;
    static float Gain;
    static int PlaneConstraint;

    static void _clear();

    static std::string VisualizerServerIP;
    static int UploadImage;
};
}  // namespace DeltaVins
