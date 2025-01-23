#include "framework/slamAPI.h"

#include "Algorithm/vision/camModel/camModel.h"
#include "IO/dataBuffer/imuBuffer.h"
#include "IO/dataSource/dataSource.h"
#include "IO/dataSource/dataSource_Euroc.h"
#include "framework/VIOModule.h"
#include "precompile.h"
#include "utils/Config.h"
#if ENABLE_VISUALIZER
#include "../../SlamVisualizer/SlamVisualizer.h"
#elif ENABLE_VISUALIZER_TCP
#include <IO/dataOuput/PoseOutputTCP.h>
#endif

#include <IO/dataOuput/DataOutputROS.h>
#include <IO/dataOuput/DataRecorder.h>

#include "IO/dataSource/dataSource_Synthetic.h"
#include "utils/TickTock.h"

#if USE_ROS2
#include "IO/dataSource/dataSource_ROS2.h"
#endif

using namespace DeltaVins;

DataSource::Ptr dataSourcePtr = nullptr;
VIOModule::Ptr vioModulePtr = nullptr;
DataRecorder::Ptr dataRecorderPtr = nullptr;
#if ENABLE_VISUALIZER
SlamVisualizer::Ptr slamVisualizerPtr = nullptr;
#elif ENABLE_VISUALIZER_TCP
PoseOutputTcp::Ptr tcpPtr = nullptr;
#endif

#if USE_ROS2
DataOutputROS::Ptr rosPtr = nullptr;
#endif

void InitSlamSystem(const char* configFile) {
    // Init Log
    logInit();

    // load config file
    Config::loadConfigFile(configFile);
    CamModel::loadCalibrations();
    if (Config::CameraCalibration) return;

        // Init DataSource
#ifndef USE_ROS2
    if (Config::DataSourceType == DataSrcEuroc)
        dataSourcePtr = std::static_pointer_cast<DataSource>(
            std::make_shared<DataSource_Euroc>());
    else if (Config::DataSourceType == DataSrcSynthetic)
        dataSourcePtr = std::static_pointer_cast<DataSource>(
            std::make_shared<DataSource_Synthetic>());
#else
    if (Config::DataSourceType == DataSrcROS2)
        dataSourcePtr = std::static_pointer_cast<DataSource>(
            std::make_shared<DataSource_ROS2>());
#endif
    else
        LOGE("Unknown DataSourceType");

    // Init VIO Module

    if (Config::RunVIO) vioModulePtr = std::make_shared<VIOModule>();

    if (Config::RecordData) {
        dataRecorderPtr = std::make_shared<DataRecorder>();
        dataSourcePtr->AddImageObserver(dataRecorderPtr.get());
    }

#if ENABLE_VISUALIZER
    if (!Config::NoGUI) {
        slamVisualizerPtr = std::make_shared<SlamVisualizer>(640, 480);
        if (Config::RunVIO) {
            vioModulePtr->SetFrameAdapter(slamVisualizerPtr.get());
            vioModulePtr->SetPointAdapter(slamVisualizerPtr.get());
        } else if (Config::RecordData) {
            dataRecorderPtr->AddFrameAdapter(slamVisualizerPtr.get());
            dataRecorderPtr->AddWorldPointAdapter(slamVisualizerPtr.get());
        }
    }
#elif ENABLE_VISUALIZER_TCP
    if (!Config::NoGUI) {
        tcpPtr = std::make_shared<PoseOutputTcp>();
        if (Config::RunVIO) {
            vioModulePtr->SetFrameAdapter(tcpPtr.get());
            vioModulePtr->SetPointAdapter(tcpPtr.get());
        } else if (Config::RecordData) {
            dataRecorderPtr->AddFrameAdapter(tcpPtr.get());
            dataRecorderPtr->AddWorldPointAdapter(tcpPtr.get());
        }
    }
#elif USE_ROS2
    if (!Config::NoGUI) {
        rosPtr = std::make_shared<DataOutputROS>();
        if (Config::RunVIO) {
            vioModulePtr->SetFrameAdapter(rosPtr.get());
            vioModulePtr->SetPointAdapter(rosPtr.get());
        } 
   }
#endif

    // add links between modules
    if (Config::RunVIO) dataSourcePtr->AddImageObserver(vioModulePtr.get());
    dataSourcePtr->AddImuObserver(&ImuBuffer::Instance());
}

void StartAndJoin() {
    if (Config::CameraCalibration) return;

    // Start all modules
    if (Config::RunVIO) {
        LOGI("Start VIO");
        vioModulePtr->Start();
    }
    if (Config::RecordData) {
        LOGI("Start Record Data\"");
        dataRecorderPtr->Start();
    }
#if ENABLE_VISUALIZER
    if (!Config::NoGUI && slamVisualizerPtr) slamVisualizerPtr->Start();
#endif
    // Join
    dataSourcePtr->Start();
#if USE_ROS2
    rclcpp::spin(std::static_pointer_cast<rclcpp::Node>(
        std::static_pointer_cast<DataSource_ROS2>(dataSourcePtr)));
#endif

    dataSourcePtr->Join();

#if ENABLE_VISUALIZER
    if (slamVisualizerPtr) slamVisualizerPtr->Join();
#endif
}

void StopSystem() {
#if USE_ROS2
    rclcpp::shutdown();
#endif
    if (Config::CameraCalibration) return;
    TickTock::outputResult();
    dataSourcePtr->Stop();
    if (Config::RunVIO) vioModulePtr->Stop();
    finishLogging();
}