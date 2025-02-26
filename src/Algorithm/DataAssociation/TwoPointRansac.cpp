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
#include "Algorithm/DataAssociation/TwoPointRansac.h"

#include "Algorithm/vision/camModel/camModel.h"
#include "utils/SensorConfig.h"
#include "precompile.h"
#include "utils/utils.h"

namespace DeltaVins {

TwoPointRansac::TwoPointRansac(float max_reproject_err, int max_iter_num) {
    max_iter_num_ = max_iter_num;
    max_sqr_reproj_err_ = max_reproject_err * max_reproject_err;
}

void TwoPointRansac::ComputeEssentialMatrix() {
    auto& ray1_0 = (*ray1_)[sample0_index_];
    auto& ray10_0 = ray10_[sample0_index_];
    auto& ray1_1 = (*ray1_)[sample1_index_];
    auto& ray10_1 = ray10_[sample1_index_];

    float ray1_0_x = ray1_0.x(), ray1_0_y = ray1_0.y(), ray1_0_z = ray1_0.z();
    float ray10_0_x = ray10_0.x(), ray10_0_y = ray10_0.y(),
          ray10_0_z = ray10_0.z();
    float ray1_1_x = ray1_1.x(), ray1_1_y = ray1_1.y(), ray1_1_z = ray1_1.z();
    float ray10_1_x = ray10_1.x(), ray10_1_y = ray10_1.y(),
          ray10_1_z = ray10_1.z();

    // formulation (10)
    float c1 = ray10_0_x * ray1_0_y - ray1_0_x * ray10_0_y;
    float c2 = -ray1_0_y * ray10_0_z + ray10_0_y * ray1_0_z;
    float c3 = -ray1_0_x * ray10_0_z + ray10_0_x * ray1_0_z;
    float c4 = ray10_1_x * ray1_1_y - ray1_1_x * ray10_1_y;
    float c5 = -ray1_1_y * ray10_1_z + ray10_1_y * ray1_1_z;
    float c6 = -ray1_1_x * ray10_1_z + ray10_1_x * ray1_1_z;

    // formulation (9)
    float alpha = -atan2(c4 * c2 - c1 * c5, c4 * c3 - c1 * c6);
    float beta = -atan2(c1, c2 * cos(alpha) + c3 * sin(alpha));
    // formulation (7)
    mE_ << 0, -cos(beta), -sin(beta) * sin(alpha), cos(beta), 0,
        -sin(beta) * cos(alpha), sin(beta) * sin(alpha), sin(beta) * cos(alpha),
        0;
}

int TwoPointRansac::SelectInliers(std::vector<bool>& inliers, int sensor_id) {
    static float maxSqrErr = max_sqr_reproj_err_;
    int nInliers = 0;
    CamModel::Ptr camModel = SensorConfig::Instance().GetCamModel(sensor_id);
    static const float focal2 = camModel->focal() * camModel->focal();
    for (int i = 0; i < num_points_; ++i) {
        Eigen::Vector3f l1 = mE_ * ray10_[i];
        float a0 = (*ray1_)[i].dot(l1);
        float s0 = focal2 / (l1.x() * l1.x() + l1.y() * l1.y());

        Eigen::Vector3f l2 = mE_.transpose() * (*ray1_)[i];
        float a1 = ray10_[i].dot(l2);
        float s1 = focal2 / (l2.x() * l2.x() + l2.y() * l2.y());

        reproj_err_[i] = std::max(a0 * a0 * s0, a1 * a1 * s1);

        if (reproj_err_[i] < maxSqrErr) {
            inliers[i] = true;
            nInliers++;
        } else
            inliers[i] = false;
    }
    return nInliers;
}

int TwoPointRansac::FindInliers(const std::vector<Eigen::Vector2f>& px0,
                                const std::vector<Eigen::Vector3f>& ray0,
                                const std::vector<Eigen::Vector2f>& px1,
                                const std::vector<Eigen::Vector3f>& ray1,
                                const Eigen::Matrix3f& dR,
                                std::vector<bool>& inliers, int sensor_id) {
    num_points_ = ray0.size();
    if (num_points_ < 2) {
        inliers.assign(num_points_, false);
        return 0;
    }
    mdR_ = dR;
    ray0_ = &ray0;
    ray1_ = &ray1;
    px0_ = &px0;
    px1_ = &px1;
    ray10_.resize(num_points_);
    reproj_err_.resize(num_points_);
    sample_index_ = 0;

    std::vector<bool> vInliers(num_points_);

    int nMaxInliers = -1;
    for (int i = 0; i < num_points_; ++i) {
        ray10_[i] = mdR_ * ray0[i];
    }
    for (int it = 0, n = max_iter_num_; it < n; ++it) {
        if (!NextSample()) break;
        ComputeEssentialMatrix();
        const int nInliers = SelectInliers(vInliers, sensor_id);
        if (nInliers > nMaxInliers) {
            inliers = vInliers;
            nMaxInliers = nInliers;
            n = UpdateIterNum(nInliers);
        }
    }

    return nMaxInliers;
}

int TwoPointRansac::UpdateIterNum(int nInliers) {
    return log(1 - confidence_) /
           log(1 - (1 - double(num_points_ - nInliers) / num_points_) *
                       (1 - double(num_points_ - nInliers) / num_points_));
}

bool TwoPointRansac::NextSample() {
    if (sample_index_ >= num_samples_) return false;
    sample0_index_ = randLists[sample_index_ * 2] % num_points_;
    sample1_index_ = randLists[sample_index_ * 2 + 1] % num_points_;
    sample_index_++;
    if (!IsGoodSample()) NextSample();
    return true;
}

bool TwoPointRansac::IsGoodSample() {
    const float thresh4Sample = 20.f * 20.f;
    return ((*px0_)[sample0_index_] - (*px0_)[sample1_index_]).squaredNorm() >
           thresh4Sample;
}
}  // namespace DeltaVins
