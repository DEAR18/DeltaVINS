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

        computeInvPoly();
    }

   public:
    static EquiDistantModel* createFromConfig(cv::FileStorage& config,
                                              bool right=false) {
        cv::Mat K;
        cv::Mat D;
        if (right) {
            config["Intrinsic_right"] >> K;
            config["Distortion_right"] >> D;
        } else {
            config["Intrinsic"] >> K;
            config["Distortion"] >> D;
        }
        auto pK = K.ptr<double>();
        auto pD = D.ptr<double>();
        return new EquiDistantModel(pK[0], pK[1], pK[2], pK[3], pK[4], pK[5],
                                    pD[0], pD[1], pD[2], pD[3]);
    }

    static void calibrate(cv::FileStorage& config);

    Vector3f imageToCam(const Vector2f& px) override;
    Vector2f camToImage(const Vector3f& pCam, Matrix23f& J23) override;
    Vector2f camToImage(const Vector3f& pCam) override;

    float focal() override;

   private:
    float k[4];
    float invK[10];
    float fx, fy;
    float cx, cy;
    cv::Mat K;
    cv::Mat D;
    cv::Mat unmapx, unmapy;

    void computeInvPoly();
};

}  // namespace DeltaVins