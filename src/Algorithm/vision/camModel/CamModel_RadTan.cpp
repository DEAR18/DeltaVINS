#include <precompile.h>
#include <Algorithm/vision/camModel/camModel_RadTan.h>


namespace DeltaVins
{

	RadTanModel* RadTanModel::createFromConfig(cv::FileStorage& config)
	{
		cv::Mat K;
		cv::Mat D;
		config["Intrinsic"] >> K;
		config["Distortion"] >> D;
		auto pK = K.ptr<double>();
		auto pD = D.ptr<double>();
		return new RadTanModel(pK[0], pK[1], pK[2], pK[3], pK[4], pK[5], pD[0], pD[1], pD[2], pD[3], pD[4]);
	}

	Vector3f RadTanModel::imageToCam(const Vector2f& px)
	{
		Vector3f xyz;
		cv::Point2f uv(px.x(), px.y()), px2;
		const cv::Mat src_pt(1, 1, CV_32FC2, &uv.x);
		cv::Mat dst_pt(1, 1, CV_32FC2, &px2.x);
		undistortPoints(src_pt, dst_pt, cvK, cvD);
		xyz[0] = px2.x;
		xyz[1] = px2.y;
		xyz[2] = 1.0;
		return xyz;
	}

	float RadTanModel::focal()
	{
		return fx;
	}

	bool RadTanModel::inView(const Vector3f& pCam)
	{
		return CamModel::inView(pCam);
	}

	Vector2f RadTanModel::camToImage(const Vector3f& pCam)
	{
		Vector2f px;
		float x, y, r2, r4, r6, a1, a2, a3, cdist, xd, yd;

		x = pCam.x() / pCam.z();
		y = pCam.y() / pCam.z();
		r2 = x * x + y * y;
		r4 = r2 * r2;
		r6 = r4 * r2;
		a1 = 2 * x * y;
		a2 = r2 + 2 * x * x;
		a3 = r2 + 2 * y * y;
		cdist = 1 + d0 * r2 + d1 * r4 + d4 * r6;
		xd = x * cdist + d2 * a1 + d3 * a2;
		yd = y * cdist + d2 * a3 + d3 * a1;
		px[0] = xd * fx + cx;
		px[1] = yd * fy + cy;
		return px;
	}

	Vector2f RadTanModel::camToImage(const Vector3f& pCam, Matrix23f& J23)
	{
		Vector2f px;
		float x, y, r, r2, r3, r4, r5, r6, a1, a2, a3, cdist, xd, yd;
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
		cdist = 1 + d0 * r2 + d1 * r4 + d4 * r6;
		xd = x * cdist + d2 * a1 + d3 * a2;
		yd = y * cdist + d2 * a3 + d3 * a1;

		px[0] = xd * fx + cx;
		px[1] = yd * fy + cy;

		Matrix2f J22;

		float t1 = (2 * d0 * r + 4 * d1 * r3 + 6 * d4 * r5);
		float t2 = t1 * x + 2 * d3 * r;
		float t3 = t1 * y + 2 * d2 * r;
		float t4 = 2 * d2 * y + 4 * d3 * x + cdist;
		float t5 = 4 * d2 * y + 2 * d3 * x + cdist;

		J22(0, 0) = t2 * drdx + t4;
		J22(0, 1) = t2 * drdy + 2 * d2 * x;
		J22(1, 0) = t3 * drdx + 2 * d3 * y;
		J22(1, 1) = t3 * drdy + t5;

		J23 << invZ * fx, 0, -fx * invZ * x,
			0, invZ * fy, -fy * invZ * y;

		J23 = J22 * J23;

		return px;
	}
}
