#pragma once
#include "utils/typedefs.h"
#include "utils/utils.h"


namespace DeltaVins {
	class CamModel
	{
	public:
		CamModel(int width, int height) : width_(width), height_(height)
		{
		};

		static void loadCalibrations();

		static void generateCalibrations(const std::string& path);

		static CamModel* getCamModel()
		{
			return camModel;
		}

	    static void loadChessboardPoints();

	    void rectifyImage(cv::Mat& image, cv::Mat& rectifyImage, int width, int height, float focal, float cx, float cy);

		virtual Vector3f imageToCam(const Vector2f& px) = 0;

		virtual Vector2f camToImage(const Vector3f& pCam) = 0;

		Vector3f imageToImu(const Vector2f& px)
		{
			return imageToCam(px);

		}

		Vector2f imuToImage(const Vector3f& pImu)
		{
			return camToImage(pImu + m_Tci);
		}

		Vector2f imuToImage(const Vector3f& pImu, Matrix23f& J23)
		{
			return camToImage(pImu + m_Tci, J23);

		}

		Vector3f getTci()
		{
			return m_Tci;
		}

		virtual Vector2f camToImage(const Vector3f& pCam, Matrix23f& J23) = 0;

		virtual float focal() = 0;

		virtual int width() { return width_; }

		virtual int height() { return height_; }
		
		const Matrix3f& getRci() { return m_Rci; }

		Vector3f& getPic() { return m_Pic; }

		virtual int area() { return width_ * height_; }

		virtual bool inView(const Vector2f& px,int border = 0)
		{
			return (px.x() >= border && px.x() < width_-border && px.y() >= border && px.y() < height_-border);
		};
		virtual bool inView(const Vector3f& pCam) { return false; };
	protected:
		int width_, height_;
		Matrix3f m_Rci;

		Vector3f m_Pic;
		Vector3f m_Tci;
		float m_fImageNoise;
		static CamModel* camModel;

		static std::vector<Vector3f> m_objectPoints;
		static std::vector<std::vector<Vector2f>> m_imagePoints;
	};
}