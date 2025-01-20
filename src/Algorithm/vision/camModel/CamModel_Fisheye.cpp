#include <Algorithm/vision/camModel/camModel_fisheye.h>
#include <precompile.h>

namespace DeltaVins {
OcamModel::OcamModel(float c_, float d_, float e_, float cx_, float cy_,
                     float a0_, float a2_, float a3_, float a4_)
    : cx(cx_), cy(cy_), c(c_), d(d_), e(e_) {
    poly[0] = a0_;
    poly[1] = 0;
    poly[2] = a2_;
    poly[3] = a3_;
    poly[4] = a4_;
}

OcamModel::OcamModel(float c_, float d_, float e_, float cx_, float cy_,
                     float a0_, float a2_, float a3_, float a4_, float ia0_,
                     float ia1_, float ia2_, float ia3_, float ia4_)
    : cx(cx_), cy(cy_), c(c_), d(d_), e(e_) {
    poly[0] = a0_;
    poly[1] = 0;
    poly[2] = a2_;
    poly[3] = a3_;
    poly[4] = a4_;

    inv_poly[0] = ia0_;
    inv_poly[1] = ia1_;
    inv_poly[2] = ia2_;
    inv_poly[3] = ia3_;
    inv_poly[4] = ia4_;
}

Eigen::VectorXd polyfit(Eigen::VectorXd& xVec, Eigen::VectorXd& yVec,
                        int poly_order) {
    assert(poly_order > 0);
    assert(xVec.size() > poly_order);
    assert(xVec.size() == yVec.size());

    Eigen::MatrixXd A(xVec.size(), poly_order + 1);
    Eigen::VectorXd B(xVec.size());

    for (int i = 0; i < xVec.size(); ++i) {
        const double x = xVec(i);
        const double y = yVec(i);

        double x_pow_k = 1.0;

        for (int k = 0; k <= poly_order; ++k) {
            A(i, k) = x_pow_k;
            x_pow_k *= x;
        }

        B(i) = y;
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd x = svd.solve(B);

    return x;
}

FisheyeModel::FisheyeModel(int width, int height, float cx, float cy, float c,
                           float d, float e, float a0, float a2, float a3,
                           float a4, bool alignment)
    : CamModel(width, height),
      ocamModel(c, d, e, cx, cy, a0, a2, a3, a4),
      alignment_(alignment) {
    computeInvPoly();
    fx_ = computeErrorMultiplier();
}

FisheyeModel::FisheyeModel(int width, int height, float cx, float cy, float c,
                           float d, float e, float a0, float a2, float a3,
                           float a4, float ia0, float ia1, float ia2, float ia3,
                           float ia4, bool alignment)
    : CamModel(width, height),
      ocamModel(c, d, e, cx, cy, a0, a2, a3, a4, ia0, ia1, ia2, ia3, ia4),
      alignment_(alignment) {
    fx_ = computeErrorMultiplier();
}

FisheyeModel* FisheyeModel::createFromConfig(cv::FileStorage& config) {
    cv::Mat K;
    cv::Mat D;
    config["Intrinsic"] >> K;
    config["Distortion"] >> D;
    auto pK = K.ptr<double>();
    auto pD = D.ptr<double>();
    int alignment;
    config["Alignment"] >> alignment;
    FisheyeModel* p = nullptr;
    if (D.cols == 7)
        p = new FisheyeModel(pK[0], pK[1], pK[2], pK[3], pD[0], pD[1], pD[2],
                             pD[3], pD[4], pD[5], pD[6], alignment);
    else if (D.cols == 12)
        p = new FisheyeModel(pK[0], pK[1], pK[2], pK[3], pD[0], pD[1], pD[2],
                             pD[3], pD[4], pD[5], pD[6], pD[7], pD[8], pD[9],
                             pD[10], pD[11], alignment);
    assert(p && "Failed to load Fisheye Config");
    return p;
}

void FisheyeModel::computeInvPoly() {
    std::vector<double> rou_vec;
    std::vector<double> z_vec;
    for (double rou = 0.0; rou <= (width_ + height_) / 2; rou += 0.1) {
        double rou_pow_k = 1.0;
        double z = 0.0;

        for (int k = 0; k < 5; k++) {
            z += rou_pow_k * ocamModel.poly[k];
            rou_pow_k *= rou;
        }

        rou_vec.push_back(rou);
        z_vec.push_back(z);
    }

    assert(rou_vec.size() == z_vec.size());
    Eigen::VectorXd thetas(rou_vec.size());
    Eigen::VectorXd rous(rou_vec.size());

    for (size_t i = 0; i < rou_vec.size(); ++i) {
        thetas(i) = std::atan2(z_vec.at(i), rou_vec.at(i));
        rous(i) = rou_vec.at(i);
    }

    // use lower order poly to eliminate over-fitting cause by noisy/inaccurate
    // data
    const int poly_fit_order = 4;
    Eigen::VectorXd inv_poly_coeff = polyfit(thetas, rous, poly_fit_order);

    for (int i = 0; i <= poly_fit_order; ++i) {
        ocamModel.inv_poly[i] = inv_poly_coeff(i);
    }

    printf("Inv Poly:%f %f %f %f %f\n", ocamModel.inv_poly[0],
           ocamModel.inv_poly[1], ocamModel.inv_poly[2], ocamModel.inv_poly[3],
           ocamModel.inv_poly[4]);
    std::vector<float> err;
    float sum_err = 0.f;
    for (int i = 0; i < height_; ++i) {
        for (int j = 0; j < width_; ++j) {
            Vector2f px0(j, i);
            Vector3f ray = imageToCam(px0);
            Vector2f px = camToImage(ray);
            float pxErr = (px0 - px).norm();
            err.push_back(pxErr);
            sum_err += pxErr;
        }
    }
    printf(" Mean Fit err:%lf\n", sum_err / err.size());

    fflush(stdout);
}

Vector3d FisheyeModel::imageToCam(const Vector2d& px) {
    Vector3d xyz;

    // Important: we exchange x and y since regular pinhole model is working
    // with x along the columns and y along the rows Davide's framework is doing
    // exactly the opposite
    if (alignment_) {
        double invdet = 1 / (ocamModel.c - ocamModel.d * ocamModel.e);

        xyz[0] = invdet * ((px.x() - ocamModel.cx) -
                           ocamModel.d * (px.y() - ocamModel.cy));
        xyz[1] = invdet * (-ocamModel.e * (px.x() - ocamModel.cx) +
                           ocamModel.c * (px.y() - ocamModel.cy));
    } else {
        xyz[0] = px.x() - ocamModel.cx;
        xyz[1] = px.y() - ocamModel.cy;
    }
    double rho =
        xyz.head<2>()
            .norm();  // distance [pixels] of  the point from the image center
    xyz[2] = ocamModel.poly[0];
    double r_i = 1;

    for (int i = 1; i < 5; i++) {
        r_i *= rho;
        xyz[2] += r_i * ocamModel.poly[i];
    }

    xyz.normalize();

    return xyz;
}

Vector3f FisheyeModel::imageToCam(const Vector2f& px) {
    Vector3f xyz;

    // Important: we exchange x and y since regular pinhole model is working
    // with x along the columns and y along the rows Davide's framework is doing
    // exactly the opposite
    if (alignment_) {
        float invdet = 1 / (ocamModel.c - ocamModel.d * ocamModel.e);

        xyz[0] = invdet * ((px.x() - ocamModel.cx) -
                           ocamModel.d * (px.y() - ocamModel.cy));
        xyz[1] = invdet * (-ocamModel.e * (px.x() - ocamModel.cx) +
                           ocamModel.c * (px.y() - ocamModel.cy));
    } else {
        xyz[0] = px.x() - ocamModel.cx;
        xyz[1] = px.y() - ocamModel.cy;
    }
    float rho =
        xyz.head<2>()
            .norm();  // distance [pixels] of  the point from the image center
    xyz[2] = ocamModel.poly[0];
    float r_i = 1;

    for (int i = 1; i < 5; i++) {
        r_i *= rho;
        xyz[2] += r_i * ocamModel.poly[i];
    }

    xyz.normalize();

    return xyz;
}

float FisheyeModel::focal() { return fx_; }

bool FisheyeModel::inView(const Vector3f& pCam) {
    return CamModel::inView(pCam);
}

Vector2d FisheyeModel::camToImage(const Vector3d& pCam) {
    Vector2d uv;

    double norm = sqrt(pCam.x() * pCam.x() + pCam.y() * pCam.y());
    double theta = std::atan2(pCam.z(), norm);
    double rho = 0;
    double theta_exp = 1.0f;
    Vector2d temp;
    if (norm < DBL_EPSILON) {
        temp << 0.f, 0.f;
    } else {
        for (int i = 0; i < 5; ++i) {
            rho += theta_exp * ocamModel.inv_poly[i];
            theta_exp *= theta;
        }
        temp.x() = pCam.x() / norm * rho;
        temp.y() = pCam.y() / norm * rho;
    }
    if (alignment_) {
        uv.x() = ocamModel.c * temp.x() + ocamModel.d * temp.y() + ocamModel.cx;
        uv.y() = ocamModel.e * temp.x() + temp.y() + ocamModel.cy;
    } else {
        uv.x() = temp.x() + ocamModel.cx;
        uv.y() = temp.y() + ocamModel.cy;
    }
    return uv;
}

Vector2f FisheyeModel::camToImage(const Vector3f& pCam) {
    Vector2f uv;

    float norm = sqrt(pCam.x() * pCam.x() + pCam.y() * pCam.y());
    float theta = std::atan2(pCam.z(), norm);
    float rho = 0;
    float theta_exp = 1.0f;
    Vector2f temp;
    if (norm < FLT_EPSILON) {
        temp << 0.f, 0.f;
    } else {
        for (int i = 0; i < 5; ++i) {
            rho += theta_exp * ocamModel.inv_poly[i];
            theta_exp *= theta;
        }
        temp.x() = pCam.x() / norm * rho;
        temp.y() = pCam.y() / norm * rho;
    }
    if (alignment_) {
        uv.x() = ocamModel.c * temp.x() + ocamModel.d * temp.y() + ocamModel.cx;
        uv.y() = ocamModel.e * temp.x() + temp.y() + ocamModel.cy;
    } else {
        uv.x() = temp.x() + ocamModel.cx;
        uv.y() = temp.y() + ocamModel.cy;
    }
    return uv;
}

Vector2f FisheyeModel::camToImage(const Vector3f& pCam, Matrix23f& J23) {
    Vector2f uv;

    float norm = sqrt(pCam.x() * pCam.x() + pCam.y() * pCam.y());
    float tanTheta = pCam.z() / norm;
    float theta = std::atan(tanTheta);
    float rho = 0;
    std::vector<float> theta_exp(6);
    theta_exp[0] = 1.0f;
    Vector2f temp;

    if (norm < FLT_EPSILON) {
        temp << 0.f, 0.f;
    } else {
        for (int i = 0; i < 5; ++i) {
            rho += theta_exp[i] * ocamModel.inv_poly[i];
            theta_exp[i + 1] = theta_exp[i] * theta;
        }
        temp.x() = pCam.x() / norm * rho;
        temp.y() = pCam.y() / norm * rho;
    }
    if (alignment_) {
        uv.x() = ocamModel.c * temp.x() + ocamModel.d * temp.y() + ocamModel.cx;
        uv.y() = ocamModel.e * temp.x() + temp.y() + ocamModel.cy;
    } else {
        uv.x() = temp.x() + ocamModel.cx;
        uv.y() = temp.y() + ocamModel.cy;
    }

    // compute Jacobian

    float A = rho;
    float invNorm = 1.f / norm;
    float B = 0.f;
    for (int i = 0; i < 4; ++i) {
        B += theta_exp[i] * (i + 1) * ocamModel.inv_poly[i + 1];
    }
    float norm2 = norm * norm;
    float invNorm2 = 1.f / norm2;
    float C = B / pCam.squaredNorm();
    float F = A * invNorm;
    float D = pCam.z() * C + F;
    float E = D * invNorm2;
    float xx = pCam.x() * pCam.x();
    float xy = pCam.y() * pCam.x();
    float yy = pCam.y() * pCam.y();

    float dudx = F - xx * E;
    float dudy = -xy * E;
    float dudz = pCam.x() * C;

    float dvdx = dudy;
    float dvdy = F - yy * E;
    float dvdz = pCam.y() * C;

    J23 << dudx, dudy, dudz, dvdx, dvdy, dvdz;

    if (alignment_) {
        Matrix2f A;
        A << ocamModel.c, ocamModel.d, ocamModel.e, 1;
        J23 = A * J23;
    }

    return uv;
}

float FisheyeModel::computeErrorMultiplier() {
    Vector3f vector1 = imageToCam(Vector2f(width_ * .5f, height_ * .5f));
    Vector3f vector2 = imageToCam(Vector2f(width_ * .5f + 0.5f, height_ * .5f));

    float factor1 = .5f / (vector1 - vector2).norm();

    vector1 = imageToCam(Vector2f(width_, .5f * height_));
    vector2 = imageToCam(Vector2f(-.5f + (float)width_, .5f * height_));

    float factor2 = .5f / (vector1 - vector2).norm();

    return (factor2 + factor1) * .5;
}

void FisheyeModel::testJacobian() {
    Vector2f px(width() * 0.5, height() * 0.5f);
    Vector3f ray = imageToCam(px);
    Vector3f ray2 = ray;
    float d = 0.001;
    ray2.x() += d;

    Matrix23f J0;
    Vector2f px0 = camToImage(ray, J0);
    Vector2f px2 = camToImage(ray2);

    Vector2f dPx = px2 - px0;
    float dudx = dPx.x() / d;
    float dvdx = dPx.y() / d;

    ray2 = ray;
    ray2.y() += d;

    px2 = camToImage(ray2);

    dPx = px2 - px0;
    float dudy = dPx.x() / d;
    float dvdy = dPx.y() / d;

    ray2 = ray;
    ray2.z() += d;

    px2 = camToImage(ray2);

    dPx = px2 - px0;
    float dudz = dPx.x() / d;
    float dvdz = dPx.y() / d;

    Matrix23f J2;
    J2 << dudx, dudy, dudz, dvdx, dvdy, dvdz;

    std::cout << J0 << std::endl << std::endl;
    std::cout << J2 << std::endl << std::endl;
    std::cout << J0 - J2 << std::endl << std::endl;
}
}  // namespace DeltaVins
