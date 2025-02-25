//
// Created by chenguojun on 2020/4/15.
//

#ifdef PLATFORM_LINUX

#include <FrameAdapter.h>
#include <IO/dataOuput/PoseOutputTCP.h>
#include <WorldPointAdapter.h>

#include "precompile.h"

namespace DeltaVins {

void PoseOutputTcp::PushViewMatrix(std::vector<FrameGL> &v_Rcw) {
    static std::vector<uchar> buffer;
    int32_t size = v_Rcw.size();
    static const int step = sizeof(float) * 16 + 2 * sizeof(uint32_t);

    int32_t lenght = step * size + sizeof(uint32_t) + 2;
    buffer.resize(lenght);
    buffer[0] = 0x59;
    buffer[1] = 0x47;
    uchar *p = &buffer[2];
    int32_t nl3 = htonl(size);
    memcpy(p, &nl3, sizeof(int32_t));
    p += sizeof(int32_t);
    for (int i = 0; i < size; ++i) {
        uint32_t nl1 = htonl(v_Rcw[i].m_id);
        uint32_t nl2 = htonl(v_Rcw[i].type);
        memcpy(p, &nl1, sizeof(uint32_t));
        memcpy(p + sizeof(uint32_t), &nl2, sizeof(uint32_t));
        memcpy(p + sizeof(uint32_t) * 2, v_Rcw[i].Twc.data(),
               sizeof(float) * 16);
        p += step;
    }

    LOGI("Send ViweMatrix Data:%d", size);

    tcp_client_view_matrix_.SendData(reinterpret_cast<char *>(&buffer[0]),
                                     lenght);
}

void PoseOutputTcp::PushImageTexture(unsigned char *imageTexture,
                                     const int width, const int height,
                                     const int channels,
                                     const std::string &name) {
    (void)name;
    if (!Config::UploadImage) return;

    static int counter = 0;
    counter++;
    {
        //            std::unique_lock<std::mutex> lck(image_mutex_);
        if (channels != this->channels_ || width != this->width_ ||
            height != this->height_) {
            image_ = cv::Mat(height, width, CV_8UC(channels));
        }

        memcpy(image_.data, imageTexture, height * width * channels);

        this->width_ = width;
        this->height_ = height;
        this->channels_ = channels;
        imagecv_.notify_all();
    }
}

void PoseOutputTcp::PushWorldPoint(const std::vector<WorldPointGL> &v_Point3f) {
    if (v_Point3f.empty()) return;
    static std::vector<uchar> buffer;
    int lenght = (sizeof(float) * 3 + sizeof(int32_t)) * v_Point3f.size() +
                 sizeof(int32_t) + 2;
    buffer.resize(lenght);
    buffer[0] = 0x59;
    buffer[1] = 0x47;

    int32_t size = v_Point3f.size();
    uchar *p = &buffer[2];
    LOGI("Send WorldPoint Data:%d", size);
    size = htonl(size);
    memcpy(p, &size, sizeof(int32_t));
    p += sizeof(int32_t);
    int step = sizeof(float) * 3 + sizeof(int32_t);
    for (auto &pt : v_Point3f) {
        int32_t id = htonl(pt.m_id);
        memcpy(p, &id, sizeof(int32_t));
        memcpy(p + sizeof(int32_t), pt.P.data(), sizeof(float) * 3);
        p += step;
    }

    tcp_client_world_point_.SendData(reinterpret_cast<char *>(&buffer[0]),
                                     lenght);
}

PoseOutputTcp::PoseOutputTcp() {
    std::string ip = Config::VisualizerServerIP;
    tcp_client_world_point_.InitTcpClient(ip.c_str(), 8848);
    tcp_client_view_matrix_.InitTcpClient(ip.c_str(), 8849);
    tcp_client_image_texture_.InitTcpClient(ip.c_str(), 8850);
    tcp_client_image_texture_.ConnectToServer();
    tcp_client_view_matrix_.ConnectToServer();
    tcp_client_world_point_.ConnectToServer();
    image_texture_thread_ = new std::thread([&]() { this->SendImage(); });
    LOGI("Connect to Server :%s", ip.c_str());
}

void PoseOutputTcp::SendImage() {
    static std::vector<uchar> header(10);
    static std::vector<uchar> encoderImage;
    static std::vector<uchar> end(2);
    int32_t imageLength;
    cv::Mat img2;
    while (true) {
        {
            std::unique_lock<std::mutex> lck(image_mutex_);
            imagecv_.wait(lck);
            imageLength = width_ * height_ * channels_;
            img2 = image_.clone();
        }
        cv::imencode(".jpg", img2, encoderImage);

        header[0] = 0x59;
        header[1] = 0x47;
        end[0] = 0x34;
        end[1] = 0x34;
        int32_t jpgLength = encoderImage.size();
        LOGI("Send Image Length:%d %d", imageLength, jpgLength);

        imageLength = htonl(imageLength);
        jpgLength = htonl(jpgLength);
        memcpy(&header[2], &imageLength, sizeof(int32_t));
        memcpy(&header[6], &jpgLength, sizeof(int32_t));

        tcp_client_image_texture_.SendData(reinterpret_cast<char *>(&header[0]),
                                           header.size());
        tcp_client_image_texture_.SendData(
            reinterpret_cast<char *>(&encoderImage[0]), encoderImage.size());
        tcp_client_image_texture_.SendData(reinterpret_cast<char *>(&end[0]),
                                           end.size());
    }
}

}  // namespace DeltaVins
#endif
