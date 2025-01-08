//
// Created by Administrator on 2020/3/19.
//

#include <IO/dataBuffer/imageBuffer.h>
#include <IO/dataBuffer/imuBuffer.h>
#include <IO/dataOuput/DataRecorder.h>
#include <IO/dataOuput/PoseOutputTCP.h>
#include <utils/utils.h>

#include "precompile.h"

namespace DeltaVins {

DataRecorder::DataRecorder() {
    m_datDir = Config::DataSourcePath;
    if (m_datDir.empty()) m_datDir = "./data";
    LOGI("Data Recorder Path:%s", m_datDir.c_str());
    existOrMkdir(m_datDir);

    existOrMkdir(m_datDir + "/cam0");
    existOrMkdir(m_datDir + "/imu0");
    m_imuFile = fopen((m_datDir + "/imu0/data.csv").c_str(), "w");
    m_camFile = fopen((m_datDir + "/cam0/data.csv").c_str(), "w");
    haveImage = false;
}

DataRecorder::~DataRecorder() {
    fclose(m_imuFile);
    fclose(m_camFile);
    m_thread->join();
    delete m_thread;
}

void DataRecorder::onImageReceived(ImageData::Ptr imageData) {
    //        std::lock_guard<std::mutex> lck(imageMutex);
    if (Config::RunVIO) {
        this->imageData = imageData;
        haveImage = true;
    } else {
        static ImageBuffer &buffer = ImageBuffer::getInstance();
        buffer.pushImage(imageData);
    }
    wakeUpMovers();
}

void DataRecorder::doWhatYouNeedToDo() {
    static ImuBuffer &imuBuffer = ImuBuffer::getInstance();
    static ImageBuffer &imageBuffer = ImageBuffer::getInstance();

    char fileName[100];

    static int imuTail = -1;
    if (Config::RecordIMU) {
        if (imuTail == -1) imuTail = imuBuffer.m_tail;
        for (int i = 0; i < Config::nImuPerImage + 10; ++i) {
            if (imuTail == imuBuffer.m_head) {
                break;
            }
            ImuData &data = imuBuffer._buf[imuTail];
            fprintf(m_imuFile, "%lld,%d,%d,%f,%f,%f,%f,%f,%f\n", data.timestamp,
                    data.idx, data.syncFlag, data.gyro(0), data.gyro(1),
                    data.gyro(2), data.acc(0), data.acc(1), data.acc(2));
            imuTail = imuBuffer.getDeltaIndex(imuTail, 1);
        }
        fflush(m_imuFile);
    }

    static ImageBuffer &buffer = ImageBuffer::getInstance();
    if (!Config::RunVIO) imageData = buffer.popTailImage();
    if (Config::RecordImage) {
        {
            std::lock_guard<std::mutex> lck(imageMutex);
            sprintf(fileName, "cam0/%lld.png", imageData->timestamp);
            fprintf(m_camFile, "%lld,%s\n", imageData->timestamp, fileName);
            cv::imwrite(m_datDir + "/" + fileName, imageData->image);
        }
    }
    if (!Config::NoGUI && m_frameAdapter) {
        if (imageData->image.channels() == 1)
            cv::cvtColor(imageData->image, imageData->image, CV_GRAY2BGR);
        m_frameAdapter->pushImageTexture(
            imageData->image.data, imageData->image.cols, imageData->image.rows,
            imageData->image.channels());
    }
    haveImage = false;
    fflush(m_camFile);
}

bool DataRecorder::haveThingsTodo() {
    static ImageBuffer &buffer = ImageBuffer::getInstance();
    if (Config::RunVIO)
        return haveImage;
    else
        return !buffer.empty();
}
}  // namespace DeltaVins