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
    RadTanModel(int width, int height, float fx, float fy, float cx, float cy,
                float d0, float d1, float d2, float d3, float d4,
                float fx_right, float fy_right, float cx_right, float cy_right,
                float d0_right, float d1_right, float d2_right, float d3_right,
                float d4_right)
        : CamModel(width, height),
          fx(fx),
          fy(fy),
          cx(cx),
          cy(cy),
          d0(d0),
          d1(d1),
          d2(d2),
          d3(d3),
          d4(d4),
          fx_right(fx_right),
          fy_right(fy_right),
          cx_right(cx_right),
          cy_right(cy_right),
          d0_right(d0_right),
          d1_right(d1_right),
          d2_right(d2_right),
          d3_right(d3_right),
          d4_right(d4_right) {
        cvK =
            (cv::Mat_<float>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
        cvD = (cv::Mat_<float>(1, 5) << d0, d1, d2, d3, d4);
        cvK_right = (cv::Mat_<float>(3, 3) << fx_right, 0.0, cx_right, 0.0,
                     fy_right, cy_right, 0.0, 0.0, 1.0);
        cvD_right = (cv::Mat_<float>(1, 5) << d0_right, d1_right, d2_right,
                     d3_right, d4_right);
    }

   public:
    static Ptr CreateFromConfig(const cv::FileStorage& config,
                                bool is_stereo = false);

    Vector3f imageToCam(const Vector2f& px, int cam_id = 0) override;

    float focal(int cam_id = 0) override;

    // bool inView(const Vector3f& pCam) override;

    Vector2f camToImage(const Vector3f& pCam, int cam_id = 0) override;

    Vector2f camToImage(const Vector3f& pCam, Matrix23f& J23,
                        int cam_id = 0) override;

   private:
    float fx, fy, cx, cy;
    float d0, d1, d2, d3, d4;
    float fx_right, fy_right, cx_right, cy_right;
    float d0_right, d1_right, d2_right, d3_right, d4_right;

    cv::Mat_<float> cvD;
    cv::Mat_<float> cvK;
    cv::Mat_<float> cvD_right;
    cv::Mat_<float> cvK_right;
};

}  // namespace DeltaVins