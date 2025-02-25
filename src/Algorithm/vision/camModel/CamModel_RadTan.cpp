#include <Algorithm/vision/camModel/camModel_RadTan.h>
#include <precompile.h>

namespace DeltaVins {

CamModel::Ptr RadTanModel::CreateFromConfig(const cv::FileStorage& config,
                                            bool is_stereo) {
    cv::Mat K;
    cv::Mat D;
    cv::Mat K_right;
    cv::Mat D_right;
    if (is_stereo) {
        config["Intrinsic"] >> K;
        config["Distortion"] >> D;
        config["Intrinsic_right"] >> K_right;
        config["Distortion_right"] >> D_right;
        auto* pK = K.ptr<double>();
        auto* pD = D.ptr<double>();
        auto* pK_right = K_right.ptr<double>();
        auto* pD_right = D_right.ptr<double>();
        return Ptr(new RadTanModel(
            pK[0], pK[1], pK[2], pK[3], pK[4], pK[5], pD[0], pD[1], pD[2],
            pD[3], pD[4], pK_right[2], pK_right[3], pK_right[4], pK_right[5],
            pD_right[0], pD_right[1], pD_right[2], pD_right[3], pD_right[4]));
    } else {
        config["Intrinsic"] >> K;
        config["Distortion"] >> D;
        auto* pK = K.ptr<double>();
        auto* pD = D.ptr<double>();
        return Ptr(new RadTanModel(pK[0], pK[1], pK[2], pK[3], pK[4], pK[5],
                                   pD[0], pD[1], pD[2], pD[3], pD[4]));
    }
}

Vector3f RadTanModel::imageToCam(const Vector2f& px, int cam_id) {
    cv::Mat K, D;
    if (cam_id == 0) {
        K = cvK;
        D = cvD;
    } else {
        K = cvK_right;
        D = cvD_right;
    }
    Vector3f xyz;
    cv::Point2f uv(px.x(), px.y()), px2;
    const cv::Mat src_pt(1, 1, CV_32FC2, &uv.x);
    cv::Mat dst_pt(1, 1, CV_32FC2, &px2.x);
    undistortPoints(src_pt, dst_pt, K, D);
    xyz[0] = px2.x;
    xyz[1] = px2.y;
    xyz[2] = 1.0;
    return xyz;
}

float RadTanModel::focal(int cam_id) { return cam_id == 0 ? fx : fx_right; }

// bool RadTanModel::inView(const Vector3f& pCam) {
//     return CamModel::inView(pCam);
// }

Vector2f RadTanModel::camToImage(const Vector3f& pCam, int cam_id) {
    cv::Mat K, D;
    float fx_, fy_, cx_, cy_;
    float d0_, d1_, d2_, d3_, d4_;
    if (cam_id == 0) {
        K = cvK;
        D = cvD;
        fx_ = fx;
        fy_ = fy;
        cx_ = cx;
        cy_ = cy;
        d0_ = d0;
        d1_ = d1;
        d2_ = d2;
        d3_ = d3;
        d4_ = d4;
    } else {
        K = cvK_right;
        D = cvD_right;
        fx_ = fx_right;
        fy_ = fy_right;
        cx_ = cx_right;
        cy_ = cy_right;
        d0_ = d0_right;
        d1_ = d1_right;
        d2_ = d2_right;
        d3_ = d3_right;
        d4_ = d4_right;
    }
    Vector2f px;
    float x, y, r2, r4, r6, a1, a2, a3, cdist, xd, yd;
    // float drdx, drdy;

    x = pCam.x() / pCam.z();
    y = pCam.y() / pCam.z();
    r2 = x * x + y * y;
    r4 = r2 * r2;
    r6 = r4 * r2;
    a1 = 2 * x * y;
    a2 = r2 + 2 * x * x;
    a3 = r2 + 2 * y * y;
    cdist = 1 + d0_ * r2 + d1_ * r4 + d4_ * r6;
    xd = x * cdist + d2_ * a1 + d3_ * a2;
    yd = y * cdist + d2_ * a3 + d3_ * a1;
    px[0] = xd * fx_ + cx_;
    px[1] = yd * fy_ + cy_;
    return px;
}

Vector2f RadTanModel::camToImage(const Vector3f& pCam, Matrix23f& J23,
                                 int cam_id) {
    cv::Mat K, D;
    float fx_, fy_, cx_, cy_;
    float d0_, d1_, d2_, d3_, d4_;
    if (cam_id == 0) {
        K = cvK;
        D = cvD;
        fx_ = fx;
        fy_ = fy;
        cx_ = cx;
        cy_ = cy;
        d0_ = d0;
        d1_ = d1;
        d2_ = d2;
        d3_ = d3;
        d4_ = d4;
    } else {
        K = cvK_right;
        D = cvD_right;
        fx_ = fx_right;
        fy_ = fy_right;
        cx_ = cx_right;
        cy_ = cy_right;
        d0_ = d0_right;
        d1_ = d1_right;
        d2_ = d2_right;
        d3_ = d3_right;
        d4_ = d4_right;
    }
    Matrix2f J22;
    Vector2f px;
    float r, r2, r3, r4, r5, r6, a1, a2, a3, cdist, xd, yd;
    float x, y;
    float drdx, drdy;

    float invZ = 1 / pCam.z();
    x = pCam.x() * invZ;
    y = pCam.y() * invZ;

    r2 = x * x + y * y;
    r = sqrtf(r2);
    r4 = r2 * r2;
    r6 = r4 * r2;
    r3 = r2 * r;
    r5 = r4 * r;
    drdx = 2 * x / r;
    drdy = 2 * y / r;

    a1 = 2 * x * y;
    a2 = r2 + 2 * x * x;
    a3 = r2 + 2 * y * y;
    cdist = 1 + d0_ * r2 + d1_ * r4 + d4_ * r6;
    xd = x * cdist + d2_ * a1 + d3_ * a2;
    yd = y * cdist + d2_ * a3 + d3_ * a1;

    px[0] = xd * fx_ + cx_;
    px[1] = yd * fy_ + cy_;

    float t1 = (2 * d0_ * r + 4 * d1_ * r3 + 6 * d4_ * r5);
    float t2 = t1 * x + 2 * d3_ * r;
    float t3 = t1 * y + 2 * d2_ * r;
    float t4 = 2 * d2_ * y + 4 * d3_ * x + cdist;
    float t5 = 4 * d2_ * y + 2 * d3_ * x + cdist;

    J22(0, 0) = t2 * drdx + t4;
    J22(0, 1) = t2 * drdy + 2 * d2_ * x;
    J22(1, 0) = t3 * drdx + 2 * d3_ * y;
    J22(1, 1) = t3 * drdy + t5;

    J23 << invZ * fx_, 0, -fx_ * invZ * x, 0, invZ * fy_, -fy_ * invZ * y;

    J23 = J22 * J23;

    return px;
}
}  // namespace DeltaVins
