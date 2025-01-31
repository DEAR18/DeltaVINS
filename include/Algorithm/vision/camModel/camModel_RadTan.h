#pragma once
#include <opencv2/opencv.hpp>

#include "camModel.h"

namespace DeltaVins {

class RadTanModel : public CamModel {
    RadTanModel(int width, int height, float fx, float fy, float cx, float cy,
                float d0, float d1, float d2, float d3, float d4)
        : CamModel(width, height),
          fx(fx),
          fy(fy),
          cx(cx),
          cy(cy),
          d0(d0),
          d1(d1),
          d2(d2),
          d3(d3),
          d4(d4) {
        cvK =
            (cv::Mat_<float>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
        cvD = (cv::Mat_<float>(1, 5) << d0, d1, d2, d3, d4);
    }

   public:
    static RadTanModel* createFromConfig(cv::FileStorage& config,bool right=false);

    Vector3f imageToCam(const Vector2f& px) override;

    float focal() override;

    bool inView(const Vector3f& pCam) override;

    Vector2f camToImage(const Vector3f& pCam) override;

    Vector2f camToImage(const Vector3f& pCam, Matrix23f& J23) override;

   private:
    float fx, fy, cx, cy;
    float d0, d1, d2, d3, d4;

    cv::Mat_<float> cvD;
    cv::Mat_<float> cvK;
};

}  // namespace DeltaVins