/**
 * This file is part of Delta_VIO.
 *
 * Delta_VIO is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Delta_VIO is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Delta_VIO. If not, see <http://www.gnu.org/licenses/>.
 */
#include "Algorithm/vision/camModel/camModel.h"

#include <Algorithm/vision/camModel/camModel_Equidistant.h>
#include <Algorithm/vision/camModel/camModel_fisheye.h>

#include "Algorithm/vision/camModel/camModel_Pinhole.h"
#include "Algorithm/vision/camModel/camModel_RadTan.h"
#include "precompile.h"
namespace DeltaVins {
CamModel* CamModel::camModel = nullptr;
std::vector<Vector3f> CamModel::m_objectPoints;
std::vector<std::vector<Vector2f>> CamModel::m_imagePoints;

void CamModel::loadCalibrations() {
    cv::FileStorage config;
    config.open(Config::CameraCalibFile + "/calibrations.yaml",
                cv::FileStorage::READ);

    if (Config::CameraCalibration) {
        std::string type;
        config["CamType"] >> type;

        CamModel::loadChessboardPoints();
        config.release();
        config.open(Config::CameraCalibFile + "/calibrations.yaml",
                    cv::FileStorage::WRITE);

        if (type == "Equidistant") EquiDistantModel::calibrate(config);
        return;
    }

    std::string type;
    config["CamType"] >> type;

    if (type == "Pinhole") {
        camModel = PinholeModel::createFromConfig(config);
    } else if (type == "RadTan") {
        camModel = RadTanModel::createFromConfig(config);
    } else if (type == "Fisheye") {
        camModel = FisheyeModel::createFromConfig(config);
    } else if (type == "Equidistant") {
        camModel = EquiDistantModel::createFromConfig(config);
    }

    // Transformation Matrix from imu frame to camera frame
    cv::Mat Tci;

    config["Tci"] >> Tci;
    Matrix3f Rci;

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            Rci(i, j) = Tci.at<double>(i, j);
        }
        camModel->m_Pic(i) = Tci.at<double>(i, 3);
    }
    camModel->m_Rci = Rci;
    camModel->m_Tci = camModel->m_Pic;
    camModel->m_Pic = -(camModel->m_Rci.transpose() * camModel->m_Pic);
}

void CamModel::loadChessboardPoints() {
    FILE* fimagePoints =
        fopen((Config::CameraCalibFile + "/imagePoints.txt").c_str(), "r");
    FILE* fobjPoints =
        fopen((Config::CameraCalibFile + "/worldPoints.txt").c_str(), "r");
    int nObjPoints, nImages, nImagePoints;
    fscanf(fobjPoints, "%d\n", &nObjPoints);
    fscanf(fimagePoints, "%d %d\n", &nImages, &nImagePoints);
    assert(nImagePoints == nObjPoints);
    m_objectPoints.resize(nObjPoints);
    m_imagePoints.resize(nImages);
    for (int i = 0; i < nObjPoints; ++i) {
        float x, y;
        fscanf(fobjPoints, "%f %f\n", &x, &y);
        m_objectPoints[i].x() = x;
        m_objectPoints[i].y() = y;
        m_objectPoints[i].z() = 0;
    }

    for (int i = 0; i < nImages; ++i) {
        m_imagePoints[i].resize(nImagePoints);
        for (int j = 0; j < nImagePoints; ++j) {
            float x, y;
            fscanf(fimagePoints, "%f %f\n", &x, &y);
            m_imagePoints[i][j].x() = x;
            m_imagePoints[i][j].y() = y;
        }
    }
}

void CamModel::rectifyImage(cv::Mat& image, cv::Mat& rectifyImage, int width,
                            int height, float focal, float cx, float cy) {
    assert(image.channels() == 1);
    rectifyImage = cv::Mat(height, width, CV_8U);

    auto p = rectifyImage.ptr();
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            Vector3f ray;
            ray.x() = (j - cx) / focal;
            ray.y() = (i - cy) / focal;
            ray.z() = 1;
            Matrix23f J23;
            Vector2f px = camToImage(ray);
            *p = getIntensitySubpix(image, cv::Point2f(px.x(), px.y()));
            ++p;
        }
    }
}
#if 0
	void CamModel::generateCalibrations(const std::string& path)
	{
		cv::FileStorage config(Config::CameraCalibFile + "/calibrations.yaml", cv::FileStorage::WRITE);
		config.writeComment("Camera Type: Pinhole or RadTan");
		config.write("CamType", "Pinhole");
		config.writeComment("Tci: Transformation Matrix from imu frame to camera frame");
		cv::Mat Tci(3, 4, CV_64F);
		setIdentity(Tci);
		config.write("Tci", Tci);
	}
#endif
}  // namespace DeltaVins
