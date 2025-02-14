//
// Created by Administrator on 2020/3/19.
//

#include <IO/dataBuffer/imageBuffer.h>
#include <IO/dataBuffer/imuBuffer.h>
#include <IO/dataOuput/DataRecorder.h>
#include <IO/dataOuput/PoseOutputTCP.h>
#include <utils/utils.h>
#include <utils/SensorConfig.h>

#include "precompile.h"

namespace DeltaVins {

DataRecorder::DataRecorder() {
    dat_dir_ = Config::DataSourcePath;
    if (dat_dir_.empty()) dat_dir_ = "./data";
    LOGI("Data Recorder Path:%s", dat_dir_.c_str());
    existOrMkdir(dat_dir_);

    existOrMkdir(dat_dir_ + "/cam0");
    existOrMkdir(dat_dir_ + "/imu0");
    imu_file_ = fopen((dat_dir_ + "/imu0/data.csv").c_str(), "w");
    cam_file_ = fopen((dat_dir_ + "/cam0/data.csv").c_str(), "w");
    flag_have_image_ = false;
}

DataRecorder::~DataRecorder() {
    fclose(imu_file_);
    fclose(cam_file_);
    thread_->join();
    delete thread_;
}

void DataRecorder::OnImageReceived(ImageData::Ptr imageData) {
    //        std::lock_guard<std::mutex> lck(image_mutex_);
    if (Config::RunVIO) {
        this->image_data_ = imageData;
        flag_have_image_ = true;
    } else {
        static ImageBuffer &buffer = ImageBuffer::Instance();
        buffer.PushImage(imageData);
    }
    WakeUpMovers();
}

void DataRecorder::DoWhatYouNeedToDo() {
    static ImuBuffer &imuBuffer = ImuBuffer::Instance();
    // static ImageBuffer &imageBuffer = ImageBuffer::Instance();

    char fileName[100];
    static int imu_fps = SensorConfig::Instance().GetIMUParams(0).fps;
    static int image_fps = SensorConfig::Instance().GetCameraParams(0).fps;
    static int nImuPerImage = imu_fps / image_fps;
    static int imuTail = -1;
    if (Config::RecordIMU) {
        if (imuTail == -1) imuTail = imuBuffer.tail_;
        for (int i = 0; i < nImuPerImage + 10; ++i) {
            if (imuTail == imuBuffer.head_) {
                break;
            }
            ImuData &data = imuBuffer.buf_[imuTail];
            fprintf(imu_file_, "%ld,%f,%f,%f,%f,%f,%f\n", data.timestamp,
                    data.gyro(0), data.gyro(1), data.gyro(2), data.acc(0),
                    data.acc(1), data.acc(2));
            imuTail = imuBuffer.getDeltaIndex(imuTail, 1);
        }
        fflush(imu_file_);
    }

    static ImageBuffer &buffer = ImageBuffer::Instance();
    if (!Config::RunVIO) image_data_ = buffer.PopTailImage();
    if (Config::RecordImage) {
        {
            std::lock_guard<std::mutex> lck(image_mutex_);
            sprintf(fileName, "cam0/%ld.png", image_data_->timestamp);
            fprintf(cam_file_, "%ld,%s\n", image_data_->timestamp, fileName);
            cv::imwrite(dat_dir_ + "/" + fileName, image_data_->image);
        }
    }
    if (!Config::NoGUI && frame_adapter_) {
        if (image_data_->image.channels() == 1)
            cv::cvtColor(image_data_->image, image_data_->image,
                         cv::COLOR_GRAY2BGR);
        frame_adapter_->PushImageTexture(
            image_data_->image.data, image_data_->image.cols,
            image_data_->image.rows, image_data_->image.channels());
    }
    flag_have_image_ = false;
    fflush(cam_file_);
}

bool DataRecorder::HaveThingsTodo() {
    static ImageBuffer &buffer = ImageBuffer::Instance();
    if (Config::RunVIO)
        return flag_have_image_;
    else
        return !buffer.empty();
}

}  // namespace DeltaVins