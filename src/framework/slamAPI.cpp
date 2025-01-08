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

#include <IO/dataOuput/DataRecorder.h>

#include "IO/dataSource/dataSource_Synthetic.h"
#include "utils/TickTock.h"

using namespace DeltaVins;

DataSource::Ptr dataSourcePtr = nullptr;
VIOModule::Ptr vioModulePtr = nullptr;
DataRecorder::Ptr dataRecorderPtr = nullptr;
#if ENABLE_VISUALIZER
SlamVisualizer::Ptr slamVisualizerPtr = nullptr;
#elif ENABLE_VISUALIZER_TCP
PoseOutputTcp::Ptr tcpPtr = nullptr;
#endif
void initSlamSystem(const char* datasetDir, const char* testName) {
    // Init Log
    logInit();

    // load config file
    Config::loadConfigFile(datasetDir, testName);
    CamModel::loadCalibrations();
    if (Config::CameraCalibration) return;

    // Init DataSource

    if (Config::DataSourceType == DataSrcEuroc)
        dataSourcePtr = std::static_pointer_cast<DataSource>(
            std::make_shared<DataSource_Euroc>());
    else if (Config::DataSourceType == DataSrcSynthetic)
        dataSourcePtr = std::static_pointer_cast<DataSource>(
            std::make_shared<DataSource_Synthetic>());

    // Init VIO Module

    if (Config::RunVIO) vioModulePtr = std::make_shared<VIOModule>();

    if (Config::RecordData) {
        dataRecorderPtr = std::make_shared<DataRecorder>();
        dataSourcePtr->addImageObserver(dataRecorderPtr.get());
    }

#if ENABLE_VISUALIZER
    if (!Config::NoGUI) {
        slamVisualizerPtr = std::make_shared<SlamVisualizer>(640, 480);
        if (Config::RunVIO) {
            vioModulePtr->setFrameAdapter(slamVisualizerPtr.get());
            vioModulePtr->setPointAdapter(slamVisualizerPtr.get());
        } else if (Config::RecordData) {
            dataRecorderPtr->addFrameAdapter(slamVisualizerPtr.get());
            dataRecorderPtr->addWorldPointAdapter(slamVisualizerPtr.get());
        }
    }
#elif ENABLE_VISUALIZER_TCP
    if (!Config::NoGUI) {
        tcpPtr = std::make_shared<PoseOutputTcp>();
        if (Config::RunVIO) {
            vioModulePtr->setFrameAdapter(tcpPtr.get());
            vioModulePtr->setPointAdapter(tcpPtr.get());
        } else if (Config::RecordData) {
            dataRecorderPtr->addFrameAdapter(tcpPtr.get());
            dataRecorderPtr->addWorldPointAdapter(tcpPtr.get());
        }
    }
#endif

    // add links between modules
    if (Config::RunVIO) dataSourcePtr->addImageObserver(vioModulePtr.get());
    dataSourcePtr->addImuObserver(&ImuBuffer::getInstance());
}

void startAndJoin() {
    if (Config::CameraCalibration) return;

    // start all modules
    if (Config::RunVIO) {
        LOGI("Start VIO");
        vioModulePtr->start();
    }
    if (Config::RecordData) {
        LOGI("Start Record Data\"");
        dataRecorderPtr->start();
    }
#if ENABLE_VISUALIZER
    if (!Config::NoGUI && slamVisualizerPtr) slamVisualizerPtr->start();
#endif
    // join
    dataSourcePtr->start();

    dataSourcePtr->join();

#if ENABLE_VISUALIZER
    if (slamVisualizerPtr) slamVisualizerPtr->join();
#endif
}

void stopSystem() {
    if (Config::CameraCalibration) return;
    TickTock::outputResult();
    dataSourcePtr->stop();
    if (Config::RunVIO) vioModulePtr->stop();
    finishLogging();
}