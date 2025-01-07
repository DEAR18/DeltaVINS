#pragma once

#include "camModel.h"
#include "opencv2/opencv.hpp"

namespace DeltaVins {

	class PinholeModel : public CamModel
	{
		PinholeModel(int width, int height, float fx, float fy, float cx, float cy)
			: CamModel(width, height), fx(fx), fy(fy), cx(cx), cy(cy),
			tfovx(width / (2 * fx)), tfovy(height / (2 * fy))
		{
		};
	public:

		static PinholeModel* createFromConfig(cv::FileStorage& config)
		{
			cv::Mat K;
			cv::Mat D;
			config["Intrinsic"] >> K;
			auto pK = K.ptr<double>();
			return new PinholeModel(pK[0], pK[1], pK[2], pK[3], pK[4], pK[5]);
		}
		Vector3f imageToCam(const Vector2f& px) override
		{
			return Vector3f((px.x() - cx) / fx, (px.y() - cy) / fy, 1);
		}

		Vector2f camToImage(const Vector3f& pCam) override
		{
			return Vector2f(pCam.x() / pCam.z() * fx + cx, pCam.y() / pCam.z() * fy + cy);
		}

		Vector2f camToImage(const Vector3f& pCam, Matrix23f& J23) override
		{
			float invZ = 1 / pCam.z();
			float ux = fx * pCam.x() * invZ;
			float uy = fy * pCam.y() * invZ;
			J23 << invZ * fx, 0, -invZ * ux,
				0, invZ* fy, -invZ * uy;
			return Vector2f(ux+cx, uy+cy);
		}


		float focal() override
		{
			return fx;
		}

		bool inView(const Vector3f& pCam) override
		{
			return (pCam.x() < tfovx * pCam.z() && pCam.y() < tfovy * pCam.z());
		}

	private:
		float fx, fy, cx, cy;
		float tfovx, tfovy;
	};

}