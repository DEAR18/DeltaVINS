#pragma once

#include "camModel.h"
#include "opencv2/opencv.hpp"

namespace DeltaVins {

class PinholeModel : public CamModel {
    PinholeModel(int width, int height, float fx, float fy, float cx, float cy)
        : CamModel(width, height),
          fx(fx),
          fy(fy),
          cx(cx),
          cy(cy),
          tfovx(width / (2 * fx)),
          tfovy(height / (2 * fy)) {};
    PinholeModel(int width, int height, float fx, float fy, float cx, float cy,
                 float fx_right, float fy_right, float cx_right, float cy_right)
        : CamModel(width, height),
          fx(fx),
          fy(fy),
          cx(cx),
          cy(cy),
          tfovx(width / (2 * fx)),
          tfovy(height / (2 * fy)),
          fx_right(fx_right),
          fy_right(fy_right),
          cx_right(cx_right),
          cy_right(cy_right),
          tfovx_right(width / (2 * fx_right)),
          tfovy_right(height / (2 * fy_right)) {};

   public:
    static Ptr CreateFromConfig(const cv::FileStorage& config, bool is_stereo) {
        cv::Mat K, K_right, D, D_right;
        if (is_stereo) {
            config["Intrinsic"] >> K;
            config["Intrinsic_right"] >> K_right;
            auto* pK = K.ptr<double>();
            auto* pK_right = K_right.ptr<double>();
            return Ptr(new PinholeModel(pK[0], pK[1], pK[2], pK[3], pK[4],
                                        pK[5], pK_right[2], pK_right[3],
                                        pK_right[4], pK_right[5]));
        } else {
            config["Intrinsic"] >> K;
            auto* pK = K.ptr<double>();
            return Ptr(
                new PinholeModel(pK[0], pK[1], pK[2], pK[3], pK[4], pK[5]));
        }
    }

    Vector3f imageToCam(const Vector2f& px, int cam_id = 0) override {
        if (cam_id == 0) {
            return Vector3f((px.x() - cx) / fx, (px.y() - cy) / fy, 1);
        } else {
            return Vector3f((px.x() - cx_right) / fx_right,
                            (px.y() - cy_right) / fy_right, 1);
        }
    }

    Vector2f camToImage(const Vector3f& pCam, int cam_id = 0) override {
        if (cam_id == 0) {
            return Vector2f(pCam.x() / pCam.z() * fx + cx,
                            pCam.y() / pCam.z() * fy + cy);
        } else {
            return Vector2f(pCam.x() / pCam.z() * fx_right + cx_right,
                            pCam.y() / pCam.z() * fy_right + cy_right);
        }
    }

    Vector2f camToImage(const Vector3f& pCam, Matrix23f& J23,
                        int cam_id = 0) override {
        if (cam_id == 0) {
            float invZ = 1 / pCam.z();
            float ux = fx * pCam.x() * invZ;
            float uy = fy * pCam.y() * invZ;
            J23 << invZ * fx, 0, -invZ * ux, 0, invZ * fy, -invZ * uy;
            return Vector2f(ux + cx, uy + cy);
        } else {
            float invZ = 1 / pCam.z();
            float ux = fx_right * pCam.x() * invZ;
            float uy = fy_right * pCam.y() * invZ;
            J23 << invZ * fx_right, 0, -invZ * ux, 0, invZ * fy_right,
                -invZ * uy;
            return Vector2f(ux + cx_right, uy + cy_right);
        }
    }

    float focal(int cam_id = 0) override { return cam_id == 0 ? fx : fx_right; }

    // bool inView(const Vector3f& pCam) override {
    //     return (pCam.x() < tfovx * pCam.z() && pCam.y() < tfovy * pCam.z());
    // }

   private:
    float fx, fy, cx, cy;
    float tfovx, tfovy;
    bool stereo_;
    float fx_right, fy_right, cx_right, cy_right;
    float tfovx_right, tfovy_right;
};

}  // namespace DeltaVins