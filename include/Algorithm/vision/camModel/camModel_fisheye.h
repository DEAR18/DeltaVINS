#pragma once
#include "camModel.h"

#include <opencv2/opencv.hpp>
#include <utils/log.h>

namespace DeltaVins {

#define MAX_POL_LENGTH 64

	struct OcamModel
	{
		OcamModel(float c_, float d_, float e_, float cx_, float cy_, float a0_, float a2_, float a3_, float a4_);;
		OcamModel(float c_, float d_, float e_, float cx_, float cy_, float a0_, float a2_, float a3_, float a4_,
		          float ia0_, float ia1_, float ia2_, float ia3_, float ia4_);;
		float cx;				// row coordinate of the center
		float cy;         			// column coordinate of the center
		float c;				// affine parameter
		float d;				// affine parameter
		float e;				// affine parameter
		float poly[5];   // ploy
		float inv_poly[5];//inv ploy
	};


	class FisheyeModel : public CamModel
	{
		OcamModel ocamModel;
		FisheyeModel(int width, int height, float cx, float cy, float c, float d, float e, float a0, float a2, float a3,
		             float a4, bool aligment);

		FisheyeModel(int width, int height, float cx, float cy, float c, float d, float e, float a0, float a2, float a3,
		             float a4, float ia0, float ia1, float ia2, float ia3, float ia4, bool aligment);

	public:
		static FisheyeModel* createFromConfig(cv::FileStorage& config);

		static void calibrate();

		void computeInvPoly();

		Vector3d imageToCam(const Vector2d& px);

		Vector3f imageToCam(const Vector2f& px) override;

		float focal() override;

		bool inView(const Vector3f& pCam) override;

		Vector2d camToImage(const Vector3d& pCam);

		Vector2f camToImage(const Vector3f& pCam) override;

		Vector2f camToImage(const Vector3f& pCam, Matrix23f& J23) override;

		float computeErrorMultiplier();

		void testJacobian();

	private:
        bool alignment = true;
		float fx;
	};

}