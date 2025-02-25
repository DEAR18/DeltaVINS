#pragma once
#ifdef PLATFORM_LINUX

#include <FrameAdapter.h>
#include <WorldPointAdapter.h>

#include "IO/dataOuput/yg_tcp_socket.h"

namespace DeltaVins {

class PoseOutputTcp : public WorldPointAdapter, public FrameAdapter {
   public:
    void PushViewMatrix(std::vector<FrameGL> &v_Rcw) override;

    void PushImageTexture(unsigned char *imageTexture, const int width,
                          const int height, const int channels,
                          const std::string &name) override;

    void PushWorldPoint(const std::vector<WorldPointGL> &v_Point3f) override;

    void FinishFrame() override {};

    using Ptr = std::shared_ptr<PoseOutputTcp>;
    PoseOutputTcp();

   private:
    void SendImage();

    std::thread *image_texture_thread_;
    int width_;
    int height_;
    int channels_;
    std::mutex image_mutex_;

    YG_TCP_Client tcp_client_world_point_;
    YG_TCP_Client tcp_client_image_texture_;
    YG_TCP_Client tcp_client_view_matrix_;
    std::condition_variable imagecv_;
    cv::Mat image_;
};

}  // namespace DeltaVins

#endif  // PLATFORM_LINUX
