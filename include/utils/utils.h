#pragma once
#include <vector>
#include <string>
#include "Config.h"
#include "targetDefine.h"
#include "typedefs.h"
#include "log.h"
#include "ModuleDefine.h"
#include "FileOutput.h"
#include "TickTock.h"

namespace DeltaVins
{

/*
 *Parse (split) a string in C++ using string delimiter (standard C++)
 *https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c
 */
	inline std::vector<std::string> split(const std::string& str, char delimiter)
	{
		std::vector<std::string> internal;
		std::stringstream ss(str); // Turn the string into a stream.
		std::string tok;

		while (getline(ss, tok, delimiter))
		{
			internal.push_back(tok);
		}
		return internal;
	}




	template<typename T>
	T linearInterpolate(const T& a, const T& b, float k)
	{
		return (1 - k) * a + k * b;
	}


   
    inline Quaternionf expAndTheta(const Vector3f& omega)
    {
        const float theta_sq = omega.squaredNorm();
        float theta = std::sqrt(theta_sq);
        const float half_theta = static_cast<float>(0.5)* (theta);

        float imag_factor;
        float real_factor;;
        if ((theta) < 1e-10f) {
            const float theta_po4 = theta_sq * theta_sq;
            imag_factor = static_cast<float>(0.5)
                - static_cast<float>(1.0 / 48.0)* theta_sq
                + static_cast<float>(1.0 / 3840.0)* theta_po4;
            real_factor = static_cast<float>(1)
                - static_cast<float>(0.5)* theta_sq +
                static_cast<float>(1.0 / 384.0)* theta_po4;
        }
        else {
            const float sin_half_theta = std::sin(half_theta);
            imag_factor = sin_half_theta / (theta);
            real_factor = std::cos(half_theta);
        }

        return Quaternionf(real_factor,
            imag_factor * omega.x(),
            imag_factor * omega.y(),
            imag_factor * omega.z());
    }

    inline Matrix3f crossMat(const Vector3f& x)
    {
        Matrix3f X;
        X << 0, -x(2), x(1),
            x(2), 0, -x(0),
            -x(1), x(0), 0;
        return X;
    }


    inline Matrix3f getRotFromGravAndMag(const Eigen::Vector3f& gravity, const Eigen::Vector3f& magnet)
    {
        float Ax = gravity[0];
        float Ay = gravity[1];
        float Az = gravity[2];
        float Ex = magnet[0];
        float Ey = magnet[1];
        float Ez = magnet[2];
        float Hx = Ey * Az - Ez * Ay;
        float Hy = Ez * Ax - Ex * Az;
        float Hz = Ex * Ay - Ey * Ax;
        float normH = (float)sqrt(Hx * Hx + Hy * Hy + Hz * Hz);
        if (normH < FLT_EPSILON) {
            return Matrix3f::Identity();
        }
        float invH = 1.0f / normH;
        Hx *= invH;
        Hy *= invH;
        Hz *= invH;
        float invA = 1.0f / sqrt(Ax * Ax + Ay * Ay + Az * Az);
        Ax *= invA;
        Ay *= invA;
        Az *= invA;
        float Mx = Ay * Hz - Az * Hy;
        float My = Az * Hx - Ax * Hz;
        float Mz = Ax * Hy - Ay * Hx;
        Eigen::Matrix3f R;

        R << Hx, Hy, Hz,
            -Ax, -Ay, -Az,
            Mx, My, Mz;
#if USE_Z_AXIS
        Matrix3f A;
        A << 1, 0, 0, 0, 0, 1, 0, 1, 0;
        return A * R;
#else
        return  R;
#endif
    }


    bool existOrMkdir(const std::string&dir);

    template <typename T>
    T bilinearInterploate(const T&v00,const T&v01,const T&v10,const T&v11, float a,const float b){
        return v00 * (1.f-a) + v01*(1.f-b) + v10*a + v11*b;
    }



    inline long long getTimestamp()
    {
        static timespec t;
        timespec_get(&t,TIME_UTC);
        return t.tv_nsec+t.tv_sec*1000000000;

    }

    /*
     * https://stackoverflow.com/questions/13299409/how-to-get-the-image-pixel-at-real-locations-in-opencv
     */
    inline uchar getIntensitySubpix(const cv::Mat& img, cv::Point2f pt)
    {
        assert(!img.empty());
        assert(img.channels() == 1);

        int x = (int)pt.x;
        int y = (int)pt.y;

        int x0 = cv::borderInterpolate(x, img.cols, cv::BORDER_REFLECT_101);
        int x1 = cv::borderInterpolate(x + 1, img.cols, cv::BORDER_REFLECT_101);
        int y0 = cv::borderInterpolate(y, img.rows, cv::BORDER_REFLECT_101);
        int y1 = cv::borderInterpolate(y + 1, img.rows, cv::BORDER_REFLECT_101);

        float a = pt.x - (float)x;
        float c = pt.y - (float)y;

		return  (uchar)cvRound((img.at<uchar>(y0, x0) * (1.f - a) + img.at<uchar>(y0, x1) * a) * (1.f - c)
            + (img.at<uchar>(y1, x0) * (1.f - a) + img.at<uchar>(y1, x1) * a) * c);

    }
	
    inline cv::Vec3b getColorSubpix(const cv::Mat& img, cv::Point2f pt)
    {
        assert(!img.empty());
        assert(img.channels() == 3);

        int x = (int)pt.x;
        int y = (int)pt.y;

        int x0 = cv::borderInterpolate(x, img.cols, cv::BORDER_REFLECT_101);
        int x1 = cv::borderInterpolate(x + 1, img.cols, cv::BORDER_REFLECT_101);
        int y0 = cv::borderInterpolate(y, img.rows, cv::BORDER_REFLECT_101);
        int y1 = cv::borderInterpolate(y + 1, img.rows, cv::BORDER_REFLECT_101);

        float a = pt.x - (float)x;
        float c = pt.y - (float)y;

        uchar b = (uchar)cvRound((img.at<cv::Vec3b>(y0, x0)[0] * (1.f - a) + img.at<cv::Vec3b>(y0, x1)[0] * a) * (1.f - c)
            + (img.at<cv::Vec3b>(y1, x0)[0] * (1.f - a) + img.at<cv::Vec3b>(y1, x1)[0] * a) * c);
        uchar g = (uchar)cvRound((img.at<cv::Vec3b>(y0, x0)[1] * (1.f - a) + img.at<cv::Vec3b>(y0, x1)[1] * a) * (1.f - c)
            + (img.at<cv::Vec3b>(y1, x0)[1] * (1.f - a) + img.at<cv::Vec3b>(y1, x1)[1] * a) * c);
        uchar r = (uchar)cvRound((img.at<cv::Vec3b>(y0, x0)[2] * (1.f - a) + img.at<cv::Vec3b>(y0, x1)[2] * a) * (1.f - c)
            + (img.at<cv::Vec3b>(y1, x0)[2] * (1.f - a) + img.at<cv::Vec3b>(y1, x1)[2] * a) * c);

        return cv::Vec3b(b, g, r);
    }



    extern float chi2LUT[];
    extern int randLists[];


#define _RED_SCALAR cv::Scalar(0,0,255,1)
#define _GREEN_SCALAR cv::Scalar(0,255,0,1)
#define _BLUE_SCALAR cv::Scalar(255,0,0,1)
	
}