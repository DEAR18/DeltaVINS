#pragma once
#include <opencv2/opencv.hpp>

#include "camModel.h"

namespace DeltaVins {

class EquiDistantModel : public CamModel {
   public:
    EquiDistantModel(int width, int height, float fx, float fy, float cx,
                     float cy, float k1, float k2, float k3, float k4)
        : CamModel(width, height, CamModelType::EQUIDISTANT),
          fx_(fx),
          fy_(fy),
          cx_(cx),
          cy_(cy) {
        k_[0] = k1;
        k_[1] = k2;
        k_[2] = k3;
        k_[3] = k4;
        K_ = (cv::Mat_<float>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
        D_ = (cv::Mat_<float>(1, 4) << k1, k2, k3, k4);
    }

    EquiDistantModel(int width, int height, float fx, float fy, float cx,
                     float cy, float k1, float k2, float k3, float k4,
                     float fx_right, float fy_right, float cx_right,
                     float cy_right, float k1_right, float k2_right,
                     float k3_right, float k4_right)
        : CamModel(width, height, CamModelType::EQUIDISTANT),
          fx_(fx),
          fy_(fy),
          cx_(cx),
          cy_(cy),
          fx_right_(fx_right),
          fy_right_(fy_right),
          cx_right_(cx_right),
          cy_right_(cy_right) {
        k_[0] = k1;
        k_[1] = k2;
        k_[2] = k3;
        k_[3] = k4;
        k_right_[0] = k1_right;
        k_right_[1] = k2_right;
        k_right_[2] = k3_right;
        k_right_[3] = k4_right;
        K_ = (cv::Mat_<float>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
        D_ = (cv::Mat_<float>(1, 4) << k1, k2, k3, k4);
        K_right_ = (cv::Mat_<float>(3, 3) << fx_right, 0.0, cx_right, 0.0,
                    fy_right, cy_right, 0.0, 0.0, 1.0);
        D_right_ =
            (cv::Mat_<float>(1, 4) << k1_right, k2_right, k3_right, k4_right);
    }

   public:
    static Ptr CreateFromConfig(const cv::FileStorage& config,
                                bool is_stereo = false) {
        cv::Mat K;
        cv::Mat D;
        cv::Mat K_right;
        cv::Mat D_right;
        if (is_stereo) {
            config["Intrinsic"] >> K;
            config["Distortion"] >> D;
            config["Intrinsic_right"] >> K_right;
            config["Distortion_right"] >> D_right;
            auto pK = K.ptr<double>();
            auto pD = D.ptr<double>();
            auto pK_right = K_right.ptr<double>();
            auto pD_right = D_right.ptr<double>();
            return Ptr(new EquiDistantModel(
                pK[0], pK[1], pK[2], pK[3], pK[4], pK[5], pD[0], pD[1], pD[2],
                pD[3], pK_right[2], pK_right[3], pK_right[4], pK_right[5],
                pD_right[0], pD_right[1], pD_right[2], pD_right[3]));
        } else {
            config["Intrinsic"] >> K;
            config["Distortion"] >> D;
            auto pK = K.ptr<double>();
            auto pD = D.ptr<double>();
            return Ptr(new EquiDistantModel(pK[0], pK[1], pK[2], pK[3], pK[4],
                                            pK[5], pD[0], pD[1], pD[2], pD[3]));
        }
    }

    static void calibrate(cv::FileStorage& config);

    Vector3f imageToCam(const Vector2f& px, int cam_id = 0) override;
    Vector2f camToImage(const Vector3f& pCam, Matrix23f& J23,
                        int cam_id = 0) override;
    Vector2f camToImage(const Vector3f& pCam, int cam_id = 0) override;
    // bool inView(const Vector3f& pCam) override;

    float focal(int cam_id = 0) override;

    void testModelPrecision(bool is_right);

   private:
    void computeInvPoly(bool is_right = false);

   private:
    float k_[4];
    float k_right_[4];
    float invK_[10];
    float invK_right_[10];
    float fx_, fy_;
    float cx_, cy_;
    float fx_right_, fy_right_;
    float cx_right_, cy_right_;
    cv::Mat K_;
    cv::Mat D_;
    cv::Mat K_right_;
    cv::Mat D_right_;
    cv::Mat unmapx_, unmapy_;
    cv::Mat unmapx_right_, unmapy_right_;
};

}  // namespace DeltaVins