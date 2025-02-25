#pragma once
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
#pragma once
#include <vector>

#include "utils/typedefs.h"

namespace DeltaVins {

/*
 *	2-Point-based Outlier Rejection for Camera-IMU Systems with applications to
 *Micro Aerial Vehicles http://rpg.ifi.uzh.ch/docs/ICRA14_Troiani.pdf
 */

struct TwoPointRansac {
    TwoPointRansac(float maxReprojErr = 2.f, int maxIterNum = 25);

    int FindInliers(const std::vector<Eigen::Vector2f>& px0,
                    const std::vector<Eigen::Vector3f>& ray0,
                    const std::vector<Eigen::Vector2f>& px1,
                    const std::vector<Eigen::Vector3f>& ray1,
                    const Eigen::Matrix3f& dR, std::vector<bool>& inliers,
                    int sensor_id);

    void ComputeEssentialMatrix();
    int SelectInliers(std::vector<bool>& inliers, int sensor_id);
    bool NextSample();
    bool IsGoodSample();
    int UpdateIterNum(int nInliers);

    std::vector<Eigen::Vector3f> ray10_;
    const std::vector<Eigen::Vector3f>*ray0_, *ray1_;
    const std::vector<Eigen::Vector2f>*px0_, *px1_;
    int sample0_index_, sample1_index_;

    std::vector<float> reproj_err_;
    Eigen::Matrix3f mdR_;
    Eigen::Matrix3f mE_;
    Eigen::Matrix3f mF_;

    int num_samples_ = 1000;
    int sample_index_;
    int max_iter_num_;
    std::vector<std::pair<int, int>> samples_;

    float max_sqr_reproj_err_;
    int num_points_;
    float confidence_ = 0.99;
};

}  // namespace DeltaVins
