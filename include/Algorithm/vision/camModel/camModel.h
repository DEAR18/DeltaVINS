#pragma once
#include "utils/typedefs.h"
#include "utils/utils.h"

namespace DeltaVins {
class CamModel {
   public:
    CamModel(int width, int height) : width_(width), height_(height) {};

    static void loadCalibrations();

    static void generateCalibrations(const std::string& path);

    static CamModel* getCamModel(int index = 0) {
        return index ? cam_model_right_ : cam_model_;
    }

    static void loadChessboardPoints();

    void rectifyImage(cv::Mat& image, cv::Mat& rectifyImage, int width,
                      int height, float focal, float cx, float cy);

    virtual Vector3f imageToCam(const Vector2f& px) = 0;

    virtual Vector2f camToImage(const Vector3f& pCam) = 0;

    Vector3f imageToImu(const Vector2f& px) { return imageToCam(px); }

    Vector2f imuToImage(const Vector3f& pImu) {
        return camToImage(pImu + tci_);
    }

    Vector3f camToImu(const Vector3f& pCam){
        return pCam - tci_;
    }

    Vector2f imuToImage(const Vector3f& pImu, Matrix23f& J23) {
        return camToImage(pImu + tci_, J23);
    }

    Vector3f getTci() { return tci_; }

    virtual Vector2f camToImage(const Vector3f& pCam, Matrix23f& J23) = 0;

    virtual float focal() = 0;

    virtual int width() { return width_; }

    virtual int height() { return height_; }

    const Matrix3f& getRci() { return Rci_; }

    Vector3f& getPic() { return Pic_; }

    virtual int area() { return width_ * height_; }

    virtual bool inView(const Vector2f& px, int border = 0) {
        return (px.x() >= border && px.x() < width_ - border &&
                px.y() >= border && px.y() < height_ - border);
    };
    // virtual bool inView(const Vector3f& pCam) = 0;

    bool IsStereo() { return is_stereo_; }

    float depthFromStereo(const Vector2f& px1, const Vector2f& px2) {
        return focal() * baseline_ / (px1 - px2).norm();
    }

   protected:
    int cam_id;
    size_t width_, height_;
    Matrix3f Rci_, Rci_right_;
    Vector3f Pic_, Pic_right_;
    Vector3f tci_, tci_right_;
    float image_noise_;
    static CamModel* cam_model_;
    static CamModel* cam_model_right_;

    bool is_stereo_;
    float baseline_;

    static std::vector<Vector3f> m_objectPoints;
    static std::vector<std::vector<Vector2f>> m_imagePoints;
};
}  // namespace DeltaVins