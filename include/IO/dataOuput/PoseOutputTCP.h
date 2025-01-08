#pragma once
#include <FrameAdapter.h>
#include <WorldPointAdapter.h>

#include "IO/dataOuput/yg_tcp_socket.h"
#ifdef PLATFORM_LINUX

namespace DeltaVins {

class PoseOutputTcp : public WorldPointAdapter, public FrameAdapter {
   public:
    void pushViewMatrix(std::vector<FrameGL> &v_Rcw) override;

    void pushImageTexture(unsigned char *imageTexture, const int width,
                          const int height, const int channels) override;

    void pushWorldPoint(const std::vector<WorldPointGL> &v_Point3f) override;

    void finishFrame() override {};

    using Ptr = std::shared_ptr<PoseOutputTcp>;
    PoseOutputTcp();

   private:
    void sendImage();

    std::thread *m_imageTextureThread;
    int width;
    int height;
    int channels;
    std::mutex imageMutex;

    YG_TCP_Client tcpClient_worldPoint;
    YG_TCP_Client tcpClient_imageTexture;
    YG_TCP_Client tcpClient_ViewMatrix;
    std::condition_variable imagecv;
    cv::Mat image;
};

}  // namespace DeltaVins

#endif  // PLATFORM_LINUX
