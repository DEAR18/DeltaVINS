#pragma once
#include <opencv2/opencv.hpp>

#include "camModel.h"

namespace DeltaVins {

class EquiDistantModel : public CamModel {
    EquiDistantModel(int width, int height, float fx, float fy, float cx,
                     float cy, float k1, float k2, float k3, float k4)
        : CamModel(width, height), fx(fx), fy(fy), cx(cx), cy(cy) {
        k[0] = k1;
        k[1] = k2;
        k[2] = k3;
        k[3] = k4;
        K = (cv::Mat_<float>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
        D = (cv::Mat_<float>(1, 4) << k1, k2, k3, k4);

        computeInvPoly(false);
    }

    EquiDistantModel(int width, int height, float fx, float fy, float cx,
                     float cy, float k1, float k2, float k3, float k4,
                     float fx_right, float fy_right, float cx_right,
                     float cy_right, float k1_right, float k2_right,
                     float k3_right, float k4_right)
        : CamModel(width, height),
          fx(fx),
          fy(fy),
          cx(cx),
          cy(cy),
          fx_right(fx_right),
          fy_right(fy_right),
          cx_right(cx_right),
          cy_right(cy_right) {
        k[0] = k1;
        k[1] = k2;
        k[2] = k3;
        k[3] = k4;
        k_right[0] = k1_right;
        k_right[1] = k2_right;
        k_right[2] = k3_right;
        k_right[3] = k4_right;
        K = (cv::Mat_<float>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
        D = (cv::Mat_<float>(1, 4) << k1, k2, k3, k4);
        K_right = (cv::Mat_<float>(3, 3) << fx_right, 0.0, cx_right, 0.0,
                   fy_right, cy_right, 0.0, 0.0, 1.0);
        D_right =
            (cv::Mat_<float>(1, 4) << k1_right, k2_right, k3_right, k4_right);
        computeInvPoly(false);
        computeInvPoly(true);
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
                pD[3], pK_right[0], pK_right[1], pK_right[2], pK_right[3],
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

   private:
    float k[4];
    float k_right[4];
    float invK[10];
    float invK_right[10];
    float fx, fy;
    float cx, cy;
    float fx_right, fy_right;
    float cx_right, cy_right;
    cv::Mat K;
    cv::Mat D;
    cv::Mat K_right;
    cv::Mat D_right;
    cv::Mat unmapx, unmapy;
    cv::Mat unmapx_right, unmapy_right;

    void computeInvPoly(bool is_stereo = false);
};

}  // namespace DeltaVins