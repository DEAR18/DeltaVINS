#pragma once
#include <pangolin/pangolin.h>

#include <Eigen/Dense>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

#include "FrameAdapter.h"
#include "WorldPointAdapter.h"

class SlamVisualizer : public FrameAdapter, public WorldPointAdapter {
   public:
    typedef std::shared_ptr<SlamVisualizer> Ptr;
    void PushViewMatrix(std::vector<FrameGL>& v_Rwc) override;

    void PushImageTexture(unsigned char* imageTexture, const int width,
                          const int height, const int channels) override;

    void PushWorldPoint(const std::vector<WorldPointGL>& v_Point3f) override;

    void FinishFrame() override;

    SlamVisualizer(int width, int height);
    ~SlamVisualizer();

    void Start();

    void Join();

    void Stop();

    void detach();

   private:
    void _drawWorldAxis();
    void _saveFrames();
    void render();
    void _drawImageTexture();
    void _drawGeometry();
    void _clear();
    void _drawWorldPoints();
    void _drawCameraFrustum();

    int width_, height_;
    int channels_;

    unsigned char* image_texture_ = nullptr;
    pangolin::GlTexture gl_texture_;

    std::vector<float> world_points_;
    int num_point_buffer_size_;
    int point_size_;
    std::vector<FrameGL> frames_;

    std::mutex mtx_point_;
    std::mutex mtx_frame_;
    std::mutex mtx_image_texture_;
    std::atomic_bool flag_frame_by_frame_;
    std::mutex mtx_frame_by_frame_;
    std::condition_variable cv_frame_by_frame_;

    std::thread* thread_ = nullptr;

    pangolin::OpenGlRenderState view_matrix_;
    pangolin::OpenGlRenderState follow_matrix_;
    pangolin::OpenGlMatrix T_wc;
};
