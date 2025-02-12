#pragma once
#include <utils/log.h>

#include <opencv2/opencv.hpp>

#include "camModel.h"

namespace DeltaVins {

#define MAX_POL_LENGTH 64

struct OcamModel {
    OcamModel(float c_, float d_, float e_, float cx_, float cy_, float a0_,
              float a2_, float a3_, float a4_);
    OcamModel(float c_, float d_, float e_, float cx_, float cy_, float a0_,
              float a2_, float a3_, float a4_, float ia0_, float ia1_,
              float ia2_, float ia3_, float ia4_);
    float cx;           // row coordinate of the center
    float cy;           // column coordinate of the center
    float c;            // affine parameter
    float d;            // affine parameter
    float e;            // affine parameter
    float poly[5];      // ploy
    float inv_poly[5];  // inv ploy
};

class FisheyeModel : public CamModel {
    OcamModel ocamModel, *ocamModel_right = nullptr;
    FisheyeModel(int width, int height, float cx, float cy, float c, float d,
                 float e, float a0, float a2, float a3, float a4,
                 bool aligment);

    FisheyeModel(int width, int height, float cx, float cy, float c, float d,
                 float e, float a0, float a2, float a3, float a4, float ia0,
                 float ia1, float ia2, float ia3, float ia4, bool aligment);

    FisheyeModel(int width, int height, float cx, float cy, float c, float d,
                 float e, float a0, float a2, float a3, float a4,
                 float cx_right, float cy_right, float c_right, float d_right,
                 float e_right, float a0_right, float a2_right, float a3_right,
                 float a4_right, bool aligment);

    FisheyeModel(int width, int height, float cx, float cy, float c, float d,
                 float e, float a0, float a2, float a3, float a4, float ia0,
                 float ia1, float ia2, float ia3, float ia4, float cx_right,
                 float cy_right, float c_right, float d_right, float e_right,
                 float a0_right, float a2_right, float a3_right, float a4_right,
                 float ia0_right, float ia1_right, float ia2_right,
                 float ia3_right, float ia4_right, bool aligment);

   public:
    static Ptr CreateFromConfig(const cv::FileStorage& config,
                                bool is_stereo = false);

    static void calibrate();

    void computeInvPoly(bool is_stereo = false);

    // Vector3d imageToCam(const Vector2d& px);

    Vector3f imageToCam(const Vector2f& px, int cam_id = 0) override;

    float focal(int cam_id = 0) override;

    // bool inView(const Vector3f& pCam) override;

    // Vector2d camToImage(const Vector3d& pCam, int cam_id = 0);

    Vector2f camToImage(const Vector3f& pCam, int cam_id = 0) override;

    Vector2f camToImage(const Vector3f& pCam, Matrix23f& J23,
                        int cam_id = 0) override;

    float computeErrorMultiplier(int cam_id = 0);

    void testJacobian(int cam_id = 0);

   private:
    bool alignment_ = true;
    float fx_;
    float fx_right;
};

}  // namespace DeltaVins