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
FileStorage Config::m_configFile;
float Config::ImageNoise2;
float Config::GyroNoise2;
float Config::AccNoise2;
float Config::GyroBiasNoise2;
float Config::AccBiasNoise2;
float Config::ExposureTime;
float Config::Gain;

string Config::CameraCalibFile;
string Config::outputFileName;
int Config::CameraCalibration;
int Config::NoDebugOutput;
int Config::NoResultOutput;
int Config::NoGUI;
int Config::DataFps;
string Config::testName;
int Config::RecordData;
int Config::RecordIMU;
int Config::RecordImage;
int Config::UploadImage;
int Config::RunVIO;
int Config::PlaneConstraint;

string Config::VisualizerServerIP;

void Config::loadConfigFile(const std::string& datasetDir,
                            const std::string& outputName) {
    _clear();
    std::string _configFilePath;

    _configFilePath = "Config/Config.yaml";

    m_configFile.open(_configFilePath, FileStorage::READ);
    if (!m_configFile.isOpened()) {
        throw std::runtime_error("fail to open config file");
    } else
        LOGI("Load config :%s", _configFilePath.c_str());

    m_configFile["PlaneConstraint"] >> PlaneConstraint;

    string temp;
    PlaneConstraint = false;
    m_configFile["DataSourceType"] >> temp;
    if (temp == "EUROC" || temp == "Euroc")
        DataSourceType = DataSrcEuroc;
    else if (temp == "Synthetic")
        DataSourceType = DataSrcSynthetic;
    else
        throw std::runtime_error("Unknown DataSource:" + temp);

    m_configFile["DataSourcePath"] >> DataSourcePath;

    m_configFile["ImuSampleFps"] >> nImuSample;
    m_configFile["ImageSampleFps"] >> nImageSample;
    nImuPerImage = nImuSample / nImageSample;
    if (nImuSample == 0 || nImageSample == 0 || nImuSample % nImageSample) {
        throw std::runtime_error(
            "Imu Sample Speed or Image Sample Speed Error, or Imu Sample Speed "
            "is not a multiple of Image Sample Speed.");
    }

    m_configFile["PixelNoise"] >> ImageNoise2;
    m_configFile["GyroNoise"] >> GyroNoise2;
    m_configFile["AccNoise"] >> AccNoise2;
    m_configFile["GyroBiasNoise"] >> GyroBiasNoise2;
    m_configFile["AccBiasNoise"] >> AccBiasNoise2;
    GyroNoise2 = GyroNoise2 * (GyroNoise2 * nImuSample);
    AccNoise2 = AccNoise2 * (AccNoise2 * nImuSample);
    GyroBiasNoise2 = GyroBiasNoise2 * (GyroBiasNoise2 * nImuSample);
    AccBiasNoise2 = AccBiasNoise2 * (AccBiasNoise2 * nImuSample);
    ImageNoise2 = ImageNoise2 * ImageNoise2;
    m_configFile["CameraCalibrationPath"] >> CameraCalibFile;
    m_configFile["ImageStartIdx"] >> ImageStartIdx;
    m_configFile["SerialRun"] >> SerialRun;
    m_configFile["NoGUI"] >> NoGUI;
    m_configFile["NoDebugOutput"] >> NoDebugOutput;
    m_configFile["DataFps"] >> DataFps;
    m_configFile["RecordImu"] >> RecordIMU;
    m_configFile["RecordImage"] >> RecordImage;
    m_configFile["NoResultOutput"] >> NoResultOutput;
    m_configFile["ExposureTime"] >> ExposureTime;
    m_configFile["Gain"] >> Gain;
    m_configFile["CameraCalibration"] >> CameraCalibration;
    m_configFile["VisualizerServerIP"] >> VisualizerServerIP;
    m_configFile["UploadImage"] >> UploadImage;
    m_configFile["RunVIO"] >> RunVIO;

    if (RecordImage || RecordIMU) RecordData = 1;

    if (!datasetDir.empty()) {
        DataSourcePath = datasetDir;
        if (!outputName.empty()) {
            testName = outputName;
            existOrMkdir(datasetDir + "/TestResults");
            outputFileName = datasetDir + "/TestResults/" + outputName + ".csv";
        } else {
            outputFileName = "./outputPose.csv";
        }
    }
    if (outputName.empty()) outputFileName = "./outputPose.csv";
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
