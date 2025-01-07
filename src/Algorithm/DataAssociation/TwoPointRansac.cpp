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
#include "precompile.h"
#include "Algorithm/DataAssociation/TwoPointRansac.h"
#include "Algorithm/vision/camModel/camModel.h"
#include "utils/utils.h"


namespace DeltaVins {

	TwoPointRansac::TwoPointRansac(float maxReprojErr, int maxIterNum)
	{
		m_iMaxIterNum = maxIterNum;
		m_fmaxSqrReprojErr = maxReprojErr * maxReprojErr;
	}


	void TwoPointRansac::computeEssentialMatrix()
	{

		auto& ray1_0 = (*m_vpRay1)[m_iSample0];
		auto& ray10_0 = m_vRay10[m_iSample0];
		auto& ray1_1 = (*m_vpRay1)[m_iSample1];
		auto& ray10_1 = m_vRay10[m_iSample1];

		float ray1_0_x = ray1_0.x(), ray1_0_y = ray1_0.y(), ray1_0_z = ray1_0.z();
		float ray10_0_x = ray10_0.x(), ray10_0_y = ray10_0.y(), ray10_0_z = ray10_0.z();
		float ray1_1_x = ray1_1.x(), ray1_1_y = ray1_1.y(), ray1_1_z = ray1_1.z();
		float ray10_1_x = ray10_1.x(), ray10_1_y = ray10_1.y(), ray10_1_z = ray10_1.z();

		//formulation (10)
		float c1 = ray10_0_x * ray1_0_y - ray1_0_x * ray10_0_y;
		float c2 = -ray1_0_y * ray10_0_z + ray10_0_y * ray1_0_z;
		float c3 = -ray1_0_x * ray10_0_z + ray10_0_x * ray1_0_z;
		float c4 = ray10_1_x * ray1_1_y - ray1_1_x * ray10_1_y;
		float c5 = -ray1_1_y * ray10_1_z + ray10_1_y * ray1_1_z;
		float c6 = -ray1_1_x * ray10_1_z + ray10_1_x * ray1_1_z;

		//formulation (9)
		float alpha = -atan2(c4 * c2 - c1 * c5, c4 * c3 - c1 * c6);
		float beta = -atan2(c1, c2 * cos(alpha) + c3 * sin(alpha));
		//formulation (7)
		m_mE << 0, -cos(beta), -sin(beta) * sin(alpha),
			cos(beta), 0, -sin(beta) * cos(alpha),
			sin(beta)* sin(alpha), sin(beta)* cos(alpha), 0;

	}

	int TwoPointRansac::selectInliers(std::vector<bool>& inliers)
	{
		static float maxSqrErr = m_fmaxSqrReprojErr;
		int nInliers = 0;
		static const float focal2 = std::pow(CamModel::getCamModel()->focal(), 2);
		for (int i = 0; i < m_nPoints; ++i)
		{
			Eigen::Vector3f l1 = m_mE * m_vRay10[i];
			float a0 = (*m_vpRay1)[i].dot(l1);
			float s0 = focal2 / (l1.x() * l1.x() + l1.y() * l1.y());

			Eigen::Vector3f l2 = m_mE.transpose() * (*m_vpRay1)[i];
			float a1 = m_vRay10[i].dot(l2);
			float s1 = focal2 / (l2.x() * l2.x() + l2.y() * l2.y());


			m_vReprojErr[i] = std::max(a0 * a0 * s0, a1 * a1 * s1);

			if (m_vReprojErr[i] < maxSqrErr)
			{
				inliers[i] = true;
				nInliers++;
			}
			else
				inliers[i] = false;

		}
		return nInliers;
	}

	int TwoPointRansac::findInliers(const std::vector<Eigen::Vector2f>& px0, const std::vector<Eigen::Vector3f>& ray0, const std::vector<Eigen::Vector2f>&
		px1, const std::vector<Eigen::Vector3f>& ray1, const Eigen::Matrix3f& dR, std::vector<bool>& inliers)
	{
		m_nPoints = ray0.size();
		if (m_nPoints < 2)
		{
			inliers.assign(m_nPoints, false);
			return 0;
		}
		m_mdR = dR;
		m_vpRay0 = &ray0; m_vpRay1 = &ray1;
		m_vpPx0 = &px0; m_vpPx1 = &px1;
		m_vRay10.resize(m_nPoints);
		m_vReprojErr.resize(m_nPoints);
		m_sampleIdx = 0;


		std::vector<bool> vInliers(m_nPoints);

		int nMaxInliers = -1;
		for (int i = 0; i < m_nPoints; ++i)
		{
			m_vRay10[i] = m_mdR * ray0[i];
		}
		for (int it = 0, n = m_iMaxIterNum; it < n; ++it)
		{
			if (!NextSample()) break;
			computeEssentialMatrix();
			const int nInliers = selectInliers(vInliers);
			if (nInliers > nMaxInliers)
			{
				inliers = vInliers;
				nMaxInliers = nInliers;
				n = updateIterNum(nInliers);
			}
		}

		return nMaxInliers;
	}


	int TwoPointRansac::updateIterNum(int nInliers)
	{
		return log(1 - m_confidence) / log(1 - pow(1 - double(m_nPoints - nInliers) / m_nPoints, 2));
	}

	bool TwoPointRansac::NextSample()
	{
		if (m_sampleIdx >= m_nSamples) return false;
		m_iSample0 = randLists[m_sampleIdx * 2] % m_nPoints;
		m_iSample1 = randLists[m_sampleIdx * 2 + 1] % m_nPoints;
		m_sampleIdx++;
		if (!isGoodSample())
			NextSample();
		return true;
	}

	bool TwoPointRansac::isGoodSample()
	{
		const float thresh4Sample = 20.f * 20.f;
		return ((*m_vpPx0)[m_iSample0] - (*m_vpPx0)[m_iSample1]).squaredNorm() > thresh4Sample;
	}
}
