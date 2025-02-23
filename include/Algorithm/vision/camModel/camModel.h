#pragma once
#include <memory>

#include "utils/typedefs.h"
#include "utils/utils.h"

namespace DeltaVins {
class CamModel {
   public:
    using Ptr = std::shared_ptr<CamModel>;

    static Ptr CreateFromConfig(const std::string& config_path);
    static Ptr CreateFromConfig(const cv::FileStorage& config);

    void rectifyImage(cv::Mat& image, cv::Mat& rectifyImage, int width,
                      int height, float focal, float cx, float cy,
                      int cam_id = 0);

    virtual Vector3f imageToCam(const Vector2f& px, int cam_id = 0) = 0;

    virtual Vector2f camToImage(const Vector3f& pCam, int cam_id = 0) = 0;

    Vector3f imageToImu(const Vector2f& px, int cam_id = 0) {
        return cam_id ? Rci_right_.transpose() * imageToCam(px, cam_id)
                      : Rci_.transpose() * imageToCam(px, cam_id);
    }

    Vector2f imuToImage(const Vector3f& pImu, int cam_id = 0) {
        return cam_id ? camToImage(Rci_right_ * pImu + tci_right_, cam_id)
                      : camToImage(Rci_ * pImu + tci_, cam_id);
    }

    Vector3f camToImu(const Vector3f& pCam, int cam_id = 0) {
        return cam_id ? Rci_right_.transpose() * pCam + Pc_in_i_right_
                      : Rci_.transpose() * pCam + Pc_in_i_;
    }

    Vector2f imuToImage(const Vector3f& pImu, Matrix23f& J23, int cam_id = 0) {
        if (cam_id == 0) {
            Vector2f px = camToImage(Rci_ * pImu + tci_, J23, cam_id);
            J23 = J23 * Rci_;
            return px;
        } else {
            Vector2f px =
                camToImage(Rci_right_ * pImu + tci_right_, J23, cam_id);
            J23 = J23 * Rci_right_;
            return px;
        }
    }

    Vector3f getTci(int cam_id = 0) { return cam_id ? tci_right_ : tci_; }

    virtual Vector2f camToImage(const Vector3f& pCam, Matrix23f& J23,
                                int cam_id = 0) = 0;

    virtual float focal(int cam_id = 0) = 0;

    virtual int width(int cam_id = 0) {
        (void)cam_id;
        return width_;
    }

    virtual int height(int cam_id = 0) {
        (void)cam_id;
        return height_;
    }

    const Matrix3f& getRci(int cam_id = 0) {
        return cam_id ? Rci_right_ : Rci_;
    }

    Vector3f& getPic(int cam_id = 0) {
        return cam_id ? Pc_in_i_right_ : Pc_in_i_;
    }

    virtual int area(int cam_id = 0) {
        (void)cam_id;
        return width_ * height_;
    }

    virtual bool inView(const Vector2f& px, int border = 0) {
        return (px.x() >= border && px.x() < width_ - border &&
                px.y() >= border && px.y() < height_ - border);
    };

    bool IsStereo() { return is_stereo_; }

    float depthFromStereo(const Vector2f& px1, const Vector2f& px2) {
        return focal() * baseline_ / (px1 - px2).norm();
    }

    void SetVIExtrinsic(const Matrix3f& Rvi, const Vector3f& tvi, int cam_id) {
        if (cam_id == 0) {
            Rci_ = Rvi;
            tci_ = tvi;
            Pc_in_i_ = -Rci_.transpose() * tci_;
        } else {
            Rci_right_ = Rvi;
            tci_right_ = tvi;
            Pc_in_i_right_ = -Rci_right_.transpose() * tci_right_;
        }
    }

    CamModel(int width, int height) : width_(width), height_(height) {};

   protected:
    size_t width_, height_;
    int sensor_id;
    Matrix3f Rci_, Rci_right_;
    Vector3f Pc_in_i_, Pc_in_i_right_;
    Vector3f tci_, tci_right_;
    float image_noise_;

    bool is_stereo_;
    float baseline_;
};
}  // namespace DeltaVins