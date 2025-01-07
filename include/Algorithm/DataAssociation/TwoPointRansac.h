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
#include "utils/typedefs.h"
#include <vector>

namespace DeltaVins {


	/*
	 *	2-Point-based Outlier Rejection for Camera-IMU Systems with applications to Micro Aerial Vehicles
	 *	http://rpg.ifi.uzh.ch/docs/ICRA14_Troiani.pdf
	 */

	struct TwoPointRansac
	{
		TwoPointRansac(float maxReprojErr = 2.f, int maxIterNum = 25);

		int findInliers(const std::vector<Eigen::Vector2f>& px0, const std::vector<Eigen::Vector3f>& ray0, const std::vector<Eigen::Vector2f>& px1, const std::vector<Eigen::Vector3f>& ray1, const Eigen::Matrix3f& dR, std::vector<bool>& inliers);

		void computeEssentialMatrix();
		int selectInliers(std::vector<bool>& inliers);
		bool NextSample();
		bool isGoodSample();
		int updateIterNum(int nInliers);

		std::vector<Eigen::Vector3f> m_vRay10;
		const std::vector<Eigen::Vector3f>* m_vpRay0, * m_vpRay1;
		const std::vector<Eigen::Vector2f>* m_vpPx0, * m_vpPx1;
		int m_iSample0, m_iSample1;

		std::vector<float> m_vReprojErr;
		Eigen::Matrix3f m_mdR;
		Eigen::Matrix3f m_mE;
		Eigen::Matrix3f m_mF;

		int m_nSamples = 1000;
		int m_sampleIdx;
		int m_iMaxIterNum;
		std::vector<std::pair<int, int>> m_samples;

		float m_fmaxSqrReprojErr;
		int m_nPoints;
		float m_confidence = 0.99;

	};

}
