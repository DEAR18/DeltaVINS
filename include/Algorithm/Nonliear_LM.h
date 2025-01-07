#pragma once

struct LM_Result
{
	bool bConverged;
	float cost;
	float dZMax;
	float bMax;
	void clear()
	{
		bConverged = false;
		cost = std::numeric_limits<float>::max();
		dZMax = cost;
		bMax = cost;
	}
    friend std::ostream& operator <<
            (std::ostream& s,const LM_Result&p){
	    s<<"Converged:"<<p.bConverged<<" cost:"<<p.cost<<" dzMax"<<p.dZMax<<" bMax"<<p.bMax<<std::endl;
		return s;
	}
};

template<int nDim, typename Type>
class NonLinear_LM
{
public:

	NonLinear_LM(Type Epsilon1, Type Epsilon2, Type tau, int nMaxIters, bool verbose = false)
		:m_maxB(Epsilon1), m_maxdZ(Epsilon2), m_nMaxIters(nMaxIters), m_tau(tau), m_verbose(verbose)
	{

	};
	~NonLinear_LM()
	{
	}
	void clear();
	virtual float evaluateF(bool bNewZ, float huberThresh) = 0;
	virtual bool userDefinedDecentFail() = 0;
	virtual bool userDefinedConvergeCriteria() = 0;
	void solve();

    LM_Result m_Result;

protected:
	Eigen::Matrix<Type, nDim, nDim> H, HTemp;
	Eigen::Matrix<Type, nDim, 1> b, bTemp;
	Eigen::Matrix<Type, nDim, 1> z, zNew;
	Eigen::Matrix<Type, nDim, 1> dZ;
	Type m_maxB, m_maxdZ;
	int m_nMaxIters;
	int m_Iter;
	Type m_tau;
	bool m_verbose;
};

template <int nDim, typename Type>
void NonLinear_LM<nDim, Type>::clear()
{
	H.setZero();
	b.setZero();
	HTemp.setZero();
	bTemp.setZero();
	z.setZero();
	dZ.setZero();
	m_Result.clear();
}

template <int nDim, typename Type>
void NonLinear_LM<nDim, Type>::solve()
{
	float nu = 2;

	float cost0 = evaluateF(false, 10.f);
	H = HTemp;
	b = bTemp;
	float mu = H.diagonal().maxCoeff() * m_tau;
	if (b.cwiseAbs().maxCoeff() < m_maxB) {
		m_Result.bMax = b.cwiseAbs().maxCoeff();
		m_Result.bConverged = true;
	}

	for (m_Iter = 1; !m_Result.bConverged && m_Iter < m_nMaxIters; ++m_Iter)
	{
		Eigen::Matrix<Type, nDim, nDim> H_ = H + Eigen::Matrix<Type, nDim, nDim>::Identity(3, 3) * mu;

		dZ = H_.ldlt().solve(b);

		if (dZ.norm() < m_maxdZ * (z.norm() + m_maxdZ)) {
			m_Result.bConverged = true;
			if (m_verbose)
				LOGI("\t%.6f / %.6f\n", dZ.norm(), m_maxdZ * (z.norm() + m_maxdZ));
		}

		zNew = z + dZ;

		if (m_verbose) {
			LOGI("#Iter:\t %02d\n", m_Iter);
			LOGI("\t#mu:%.6f\n", mu);
			LOGI("\t#dZ:");
			for (int i = 0; i < nDim; ++i) {
				LOGI("  %.6f", dZ[i]);
			}
			LOGI("\n\t#z:\t");
			for (int i = 0; i < nDim; ++i) {
				LOGI("  %.6f", z[i]);
			}
			LOGI("  ->  ");
			for (int i = 0; i < nDim; ++i) {
				LOGI("  %.6f", zNew[i]);
			}
			LOGI("\n");
			LOGI("\n\t#b:\t");
			for (int i = 0; i < nDim; ++i) {
				LOGI("  %.6f", b[i]);
			}
			LOGI("\n");
		}

		//compute rho for estimating decent performance
		float cost1 = evaluateF(true, m_Iter > 2 ? 3.f : 10.f);
		float rho = (cost0 - cost1) / (0.5 * dZ.dot(mu * dZ + b));
		if (m_verbose)
			LOGI("\trho:%.6f\n", rho);
		if (rho > 0 && !userDefinedDecentFail())
		{
			z = zNew;
			H = HTemp;
			b = bTemp;
			if (m_verbose) {
				LOGI("\t#cost:\t %.6f  ->  %.6f\n", cost0, cost1);
				LOGI("\t#Status: Accept\n");
			}
			cost0 = cost1;
			
			m_Result.bMax = b.cwiseAbs().maxCoeff();
			m_Result.dZMax = dZ.cwiseAbs().maxCoeff();
			m_Result.cost = cost0;
			
			if (b.cwiseAbs().maxCoeff() < m_maxB)
				m_Result.bConverged = true;
			mu = mu * std::max(1.f / 3.f, 1.f - float(std::pow(2.f * rho - 1.f, 3)));
			nu = 2;


		}
		else
		{
			mu = mu * nu;
			nu = 2 * nu;
			if (m_verbose)
				LOGI("\t #Status: Reject\n");
		}

		if (m_verbose)
			LOGI("--------------------------------------------------------------\n");

	}
 	userDefinedConvergeCriteria();
}