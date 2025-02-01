#pragma once

#include <Eigen/Dense>
#include <vector>

struct FrameGL {
    FrameGL() {};
    FrameGL(const Eigen::Matrix3f& Rwc, const Eigen::Vector3f& t, int id)
        : m_id(id), type(1) {
        Twc.setIdentity();
        Twc.topLeftCorner<3, 3>() = Rwc;
        Twc.topRightCorner<3, 1>() = t;
    };
    Eigen::Matrix4f Twc;
    int32_t m_id;
    int32_t type;
};

struct FrameAdapter {
    virtual ~FrameAdapter() = default;
    virtual void PushViewMatrix(std::vector<FrameGL>& v_Rcw) = 0;
    virtual void PushImageTexture(unsigned char* imageTexture, const int width,
                                  const int height, const int channels) = 0;
    virtual void FinishFrame() = 0;
};
