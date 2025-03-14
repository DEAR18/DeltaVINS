#include "Algorithm/vision/camModel/camModel_Equidistant.h"

#include "precompile.h"
#include "utils/constantDefine.h"
#include "utils/Config.h"

namespace DeltaVins {

// void EquiDistantModel::calibrate(cv::FileStorage& config) {
//     cv::Mat K;
//     cv::Mat D;
//     std::vector<cv::Mat> rvecs, tvecs;
//     std::vector<std::vector<cv::Point3f>> objPoints;
//     std::vector<std::vector<cv::Point2f>> imagePoints;
//     for (size_t i = 0; i < m_imagePoints.size(); ++i) {
//         objPoints.emplace_back();
//         for (size_t j = 0; j < m_objectPoints.size(); ++j) {
//             auto& opv = objPoints.back();

//             opv.emplace_back(m_objectPoints[j].x(), m_objectPoints[j].y(),
//                              m_objectPoints[j].z());
//         }
//     }
//     for (size_t i = 0; i < m_imagePoints.size(); ++i) {
//         imagePoints.emplace_back();
//         for (size_t j = 0; j < m_imagePoints[i].size(); ++j) {
//             auto& ipv = imagePoints.back();
//             ipv.emplace_back(m_imagePoints[i][j].x(),
//             m_imagePoints[i][j].y());
//         }
//     }
//     cv::fisheye::calibrate(
//         objPoints, imagePoints, cv::Size(640, 480), K, D, rvecs, tvecs,
//         cv::fisheye::CALIB_CHECK_COND |
//         cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC,
//         cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
//         100,
//                          DBL_EPSILON));

//     printf("k1:%lf,k2:%lf,k3:%lf,k4:%lf\n", D.at<double>(0), D.at<double>(1),
//            D.at<double>(2), D.at<double>(3));
//     printf("fx:%lf, fy:%lf, cx:%lf, cy:%lf,", K.at<double>(0, 0),
//            K.at<double>(1, 1), K.at<double>(0, 2), K.at<double>(1, 2));
//     cv::Mat K_(1, 6, CV_64F);
//     K_.at<double>(0) = 640;
//     K_.at<double>(1) = 480;
//     K_.at<double>(2) = K.at<double>(0, 0);
//     K_.at<double>(3) = K.at<double>(1, 1);
//     K_.at<double>(4) = K.at<double>(0, 2);
//     K_.at<double>(5) = K.at<double>(1, 2);
//     config.write("CamType", "Equidistant");
//     config.write("Intrinsic", K_);
//     config.write("Distortion", D);
// }

VectorXd polyfit(VectorXd& xVec, VectorXd& yVec) {
    int polyDegree = 5;
    assert(xVec.size() == yVec.size());

    MatrixXd A(xVec.size(), polyDegree);
    VectorXd B(xVec.size());

    for (int i = 0; i < xVec.size(); ++i) {
        const double x = xVec(i);
        const double y = yVec(i);

        for (int k = 0; k < polyDegree; ++k) {
            A(i, k) = pow(x, k);
        }

        B(i) = y;
    }

    Eigen::JacobiSVD<MatrixXd> svd(A,
                                   Eigen::ComputeThinU | Eigen::ComputeThinV);
    VectorXd x = svd.solve(B);

    std::cout << A << std::endl;
    std::cout << B << std::endl;
    std::cout << A * x - B << std::endl;

    std::vector<float> errs;
    float sum_err = 0;
    for (int i = 0; i < xVec.size(); ++i) {
        const double xa = xVec(i);
        double r = 0;
        double x_exp = 1.;
        for (int k = 0; k < polyDegree; ++k) {
            r += x_exp * x(k);
            x_exp *= xa;
        }
        errs.push_back(fabs(r - yVec(i)));
        sum_err += errs.back();
    }
    std::cout << "Fit Err:" << sum_err / errs.size() << std::endl;

    return x;
}

void EquiDistantModel::computeInvPoly(bool is_right) {
    std::vector<float> r;
    std::vector<float> td;
    float* k = is_right ? k_right_ : k_;
    float* invK = is_right ? invK_right_ : invK_;
    for (float rad = 0.01; rad < PI_2; rad += 0.01) {
        float theta_d = rad * (1 + k[0] * powf(rad, 2) + k[1] * powf(rad, 4) +
                               k[2] * powf(rad, 6) + k[3] * powf(rad, 8));
        td.push_back(theta_d);
        r.push_back(tanf(rad));
    }

    VectorXd rM(r.size());
    VectorXd tdM(r.size());
    for (size_t i = 0; i < r.size(); ++i) {
        rM(i) = r[i];
        tdM(i) = td[i];
    }

    VectorXd x = polyfit(tdM, rM);
    invK[0] = x(0);
    invK[1] = x(1);
    invK[2] = x(2);
    invK[3] = x(3);
    invK[4] = x(4);

    printf("Inv Poly %f %f %f %f\n", x(0), x(1), x(2), x(3));

    std::vector<float> err;
    float sum_err = 0.f;
    for (size_t i = 0; i < height_; ++i) {
        for (size_t j = 0; j < width_; ++j) {
            Vector2f px0(j, i);
            Vector3f ray = imageToCam(px0, is_right);
            Vector2f px = camToImage(ray, is_right);
            float pxErr = (px0 - px).norm();
            err.push_back(pxErr);
            sum_err += pxErr;
            printf("%zu %zu %f\n", i, j, pxErr);
        }
    }
    printf(" Mean Fit err:%lf\n", sum_err / err.size());

    fflush(stdout);
}

float EquiDistantModel::testModelPrecision(bool is_right) {
    if (width_ == 0 || height_ == 0) {
        LOGE("Invalid camera model, width %d height %d", width_, height_);
        return 1e8f;
    }
    cv::Mat mask(
        cv::Size2f(width_, height_), CV_8UC1,
        cv::Scalar(
            255));  // set to black for pixels with large reprojection error
    float reprojection_precision = 1e-3f;
    int cnt = 0;
    float sum_err = 0.f;
    for (size_t i = 0; i < height_; ++i) {
        for (size_t j = 0; j < width_; ++j) {
            Vector2f px0(j, i);
            Vector3f ray = imageToCam(px0, is_right);
            Vector2f px = camToImage(ray, is_right);
            float pxErr = (px0 - px).norm();
            sum_err += pxErr;
            ++cnt;
            if (std::isnan(pxErr) || pxErr > reprojection_precision) {
                mask.at<uchar>(i, j) = 0;
            }
        }
    }
    float mean_err = sum_err / cnt;
    if (is_right) {
        LOGI("Right camera model type: EQUIDISTANT, mean pix->cam->pix err:%f",
             mean_err);
    } else {
        LOGI("Left camera model type: EQUIDISTANT, mean pix->cam->pix err:%f",
             sum_err / cnt);
    }
    std::string mask_name = is_right ? "/reprojection_err_mask_right.png"
                                     : "/reprojection_err_mask.png";
    cv::imwrite(Config::ResultOutputPath + mask_name, mask);

    return mean_err;
}

Vector3f EquiDistantModel::imageToCam(const Vector2f& px, int cam_id) {
    float* k = cam_id == 0 ? k_ : k_right_;
    float fx = cam_id == 0 ? fx_ : fx_right_;
    float fy = cam_id == 0 ? fy_ : fy_right_;
    float cx = cam_id == 0 ? cx_ : cx_right_;
    float cy = cam_id == 0 ? cy_ : cy_right_;
    cv::Point2f pn((px(0) - cx) / fx, (px(1) - cy) / fy);
    float scale = 1.f;
    float theta_d = sqrtf(pn.x * pn.x + pn.y * pn.y);
    theta_d = std::min(theta_d, PI_2);
    if (theta_d > 1e-8f) {
        // Compensate distortion iteratively
        float theta = theta_d;
        float precision = 1e-6f;
        for (int i = 0; i < 10; i++) {
            float theta2 = theta * theta, theta4 = theta2 * theta2,
                  theta6 = theta4 * theta2, theta8 = theta4 * theta4;
            float k0_theta2 = k[0] * theta2, k1_theta4 = k[1] * theta4,
                  k2_theta6 = k[2] * theta6, k3_theta8 = k[3] * theta8;
            float theta_fix =
                (theta * (1 + k0_theta2 + k1_theta4 + k2_theta6 + k3_theta8) -
                 theta_d) /
                (1 + 3 * k0_theta2 + 5 * k1_theta4 + 7 * k2_theta6 +
                 9 * k3_theta8);
            theta = theta - theta_fix;
            if (std::fabsf(theta_fix) < precision) break;
        }
        scale = std::tan(theta) / theta_d;
    }

    Vector3f xyz(pn.x * scale, pn.y * scale, 1.f);
    return xyz;
}

Vector2f EquiDistantModel::camToImage(const Vector3f& pCam, Matrix23f& J23,
                                      int cam_id) {
    float r = pCam.head<2>().norm();
    float theta = atan2(r, pCam.z());
    float x = pCam.x();
    float y = pCam.y();
    float z = pCam.z();
    float* k = cam_id == 0 ? k_ : k_right_;
    std::vector<float> theta_exp(10);
    theta_exp[0] = theta;
    for (size_t i = 0; i < 9; ++i) theta_exp[i + 1] = theta * theta_exp[i];
    float dtd = 1 + 3 * k[0] * theta_exp[2] + 5 * k[1] * theta_exp[4] +
                7 * k[2] * theta_exp[6] + 9 * k[3] * theta_exp[8];
    float td = theta + k[0] * theta_exp[3] + k[1] * theta_exp[5] +
               k[2] * theta_exp[7] + k[3] * theta_exp[9];
    float D = dtd / pCam.squaredNorm();

    float u = td * x / r;
    float v = td * y / r;

    float B = r * r * r;

    float A = B / z + z * r;

    float C = dtd / (r * A) - td / B;

    float dudx = td / r + x * x * C;
    float dudy = x * y * C;
    float dudz = -x * D;
    float dvdx = dudy;
    float dvdy = td / r + y * y * C;
    float dvdz = -y * D;

    float fx = cam_id == 0 ? fx_ : fx_right_;
    float fy = cam_id == 0 ? fy_ : fy_right_;
    float cx = cam_id == 0 ? cx_ : cx_right_;
    float cy = cam_id == 0 ? cy_ : cy_right_;
    J23 << dudx, dudy, dudz, dvdx, dvdy, dvdz;
    Matrix2f F;
    F << fx, 0, 0, fy;
    J23 = F * J23;

    return Vector2f(fx * u + cx, fy * v + cy);
}

Vector2f EquiDistantModel::camToImage(const Vector3f& pCam, int cam_id) {
    float norm = pCam.head<2>().norm();
    float theta = atan2(norm, pCam.z());

    float* k = cam_id == 0 ? k_ : k_right_;
    std::vector<float> theta_exp(5);
    theta_exp[0] = theta;
    float theta2 = theta * theta;
    float thetad = theta;
    for (int i = 0; i < 4; ++i) {
        theta_exp[i + 1] = theta2 * theta_exp[i];
        thetad += k[i] * theta_exp[i + 1];
    }

    float x = thetad * pCam.x() / norm;
    float y = thetad * pCam.y() / norm;

    float fx = cam_id == 0 ? fx_ : fx_right_;
    float fy = cam_id == 0 ? fy_ : fy_right_;
    float cx = cam_id == 0 ? cx_ : cx_right_;
    float cy = cam_id == 0 ? cy_ : cy_right_;
    return Vector2f(fx * x + cx, fy * y + cy);
}

float EquiDistantModel::focal(int cam_id) {
    return cam_id == 0 ? fx_ : fx_right_;
}
}  // namespace DeltaVins
