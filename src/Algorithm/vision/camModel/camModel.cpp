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
CamModel* CamModel::cam_model_ = nullptr;
CamModel* CamModel::cam_model_right_ = nullptr;
std::vector<Vector3f> CamModel::m_objectPoints;
std::vector<std::vector<Vector2f>> CamModel::m_imagePoints;

void CamModel::loadCalibrations() {
    cv::FileStorage config;
    config.open(Config::CameraCalibFile,
                cv::FileStorage::READ);

    if (Config::CameraCalibration) {
        std::string type;
        config["CamType"] >> type;

        CamModel::loadChessboardPoints();
        config.release();
        config.open(Config::CameraCalibFile,
                    cv::FileStorage::WRITE);

        if (type == "Equidistant") EquiDistantModel::calibrate(config);
        return;
    }

    std::string type;
    config["CamType"] >> type;

    bool is_stereo;
    config["IsStereo"] >> is_stereo;
    if (type == "Pinhole") {
        cam_model_ = PinholeModel::createFromConfig(config);
        if (is_stereo) {
            cam_model_right_ = PinholeModel::createFromConfig(config, true);
        }
    } else if (type == "RadTan") {
        cam_model_ = RadTanModel::createFromConfig(config);
        if (is_stereo) {
            cam_model_right_ = RadTanModel::createFromConfig(config, true);
        }
    } else if (type == "Fisheye") {
        cam_model_ = FisheyeModel::createFromConfig(config);
        if (is_stereo) {
            cam_model_right_ = FisheyeModel::createFromConfig(config, true);
        }
    } else if (type == "Equidistant") {
        cam_model_ = EquiDistantModel::createFromConfig(config);
        if (is_stereo) {
            cam_model_right_ = EquiDistantModel::createFromConfig(config, true);
        }
    }
    cam_model_->is_stereo_ = is_stereo;

    // Transformation Matrix from imu frame to camera frame
    cv::Mat Tic;

    config["Tic"] >> Tic;
    Eigen::Matrix4f Tic_eigen;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            Tic_eigen(i, j) = Tic.at<double>(i, j);
        }
    }
    Eigen::Matrix4f Tci_eigen;
    Tci_eigen.topLeftCorner<3,3>() = Tic_eigen.topLeftCorner<3,3>().transpose();
    Tci_eigen.topRightCorner<3,1>() = -Tic_eigen.topLeftCorner<3,3>().transpose() * Tic_eigen.topRightCorner<3,1>();
    cam_model_->Rci_ = Tci_eigen.block<3, 3>(0, 0);
    cam_model_->tci_ = Tci_eigen.block<3, 1>(0, 3);
    cam_model_->Pic_ = Tic_eigen.block<3, 1>(0, 3);
    if(cam_model_->is_stereo_) {
        cv::Mat Tic_right;
        config["Tic_right"] >> Tic_right;   
        Eigen::Matrix4f Tic_eigen_right;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                Tic_eigen_right(i, j) = Tic_right.at<double>(i, j);
            }
        }
        Eigen::Matrix4f Tci_eigen_right;
        Tci_eigen_right .topLeftCorner<3,3>() = Tic_eigen_right.topLeftCorner<3,3>().transpose();
        Tci_eigen_right.topRightCorner<3,1>() = -Tic_eigen_right.topLeftCorner<3,3>().transpose() * Tic_eigen_right.topRightCorner<3,1>();
        cam_model_right_->Rci_ = Tci_eigen_right.block<3, 3>(0, 0);
        cam_model_right_->tci_ = Tci_eigen_right.block<3, 1>(0, 3);
        cam_model_right_->Pic_ = Tic_eigen_right.block<3, 1>(0, 3);
        cam_model_->baseline_ = (cam_model_->Pic_ - cam_model_right_->Pic_).norm();
        cam_model_right_->baseline_ = cam_model_->baseline_; 
    }

    // Matrix3f Rci;

    // for (int i = 0; i < 3; ++i) {
    //     for (int j = 0; j < 3; ++j) {
    //         Rci(i, j) = Tci.at<double>(i, j);
    //     }
    //     cam_model_->Pic_(i) = Tci.at<double>(i, 3);
    // }
    // cam_model_->Rci_ = Rci;
    // cam_model_->tci_ = cam_model_->Pic_;
    // cam_model_->Pic_ = -(cam_model_->Rci_.transpose() * cam_model_->Pic_);

    // if (cam_model_->is_stereo_) {
    //     cv::Mat Tci_right;
    //     config["Tci_right"] >> Tci_right;
    //     Matrix3f Rci_right;

    //     for (int i = 0; i < 3; ++i) {
    //         for (int j = 0; j < 3; ++j) {
    //             Rci_right(i, j) = Tci_right.at<double>(i, j);
    //         }
    //         cam_model_right_->Pic_(i) = Tci_right.at<double>(i, 3);
    //     }
    //     cam_model_right_->Rci_ = Rci_right;
    //     cam_model_right_->tci_ = cam_model_right_->Pic_;
    //     cam_model_right_->Pic_ =
    //         -(cam_model_right_->Rci_.transpose() * cam_model_right_->Pic_);
    //     cam_model_->baseline_ = (cam_model_->Pic_ - cam_model_right_->Pic_).norm();
    //     cam_model_right_->baseline_ = cam_model_->baseline_;
    // }
}

void CamModel::loadChessboardPoints() {
    FILE* fimagePoints =
        fopen((Config::CameraCalibFile + "/imagePoints.txt").c_str(), "r");
    FILE* fobjPoints =
        fopen((Config::CameraCalibFile + "/worldPoints.txt").c_str(), "r");
    int nObjPoints, nImages, nImagePoints;
    if (fscanf(fobjPoints, "%d\n", &nObjPoints) != 1) {
        throw std::runtime_error("Failed to read number of object points");
    }
    if (fscanf(fimagePoints, "%d %d\n", &nImages, &nImagePoints) != 2) {
        throw std::runtime_error("Failed to read number of images and image points");
    }
    assert(nImagePoints == nObjPoints);
    m_objectPoints.resize(nObjPoints);
    m_imagePoints.resize(nImages);
    for (int i = 0; i < nObjPoints; ++i) {
        float x, y;
        if (fscanf(fobjPoints, "%f %f\n", &x, &y) != 2) {
            throw std::runtime_error("Failed to read object point coordinates");
        }
        m_objectPoints[i].x() = x;
        m_objectPoints[i].y() = y;
        m_objectPoints[i].z() = 0;
    }

    for (int i = 0; i < nImages; ++i) {
        m_imagePoints[i].resize(nImagePoints);
        for (int j = 0; j < nImagePoints; ++j) {
            float x, y;
            if (fscanf(fimagePoints, "%f %f\n", &x, &y) != 2) {
                throw std::runtime_error("Failed to read image point coordinates");
            }
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
