#pragma once
#include "utils/typedefs.h"


namespace DeltaVins
{
	struct ImuPreintergration
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
		long long t0;		//first data timestamp
		long long t1;		//last data timestamp
		long long dT;		//delta time

		Matrix3f dR;

		Vector3f dV;	//delta linear velocity
		Vector3f dP;	//delta position

		Matrix3f dRdg;  //jacobian rotation wrt gyroscope
		Matrix3f dVda;  //jacobian linear velocity wrt accelerator
		Matrix3f dVdg;	//jacobian linear velocity wrt gyroscope
		Matrix3f dPda;	//jacobian position wrt accelerator
		Matrix3f dPdg;	//jacobian position wrt gyroscope

		Matrix9f Cov;  //covariance matrix r,v,p

		friend std::ostream& operator <<
			(std::ostream& s,const ImuPreintergration&p)
		{
			s << "t0:" << p.t0 << "->t1:" << p.t1 << std::endl;
			s << "dR:\n" << p.dR << std::endl;
			s << "dV:\n" << p.dV << std::endl;
			s << "dRdg:\n" << p.dRdg << std::endl;
			s << "dVda:\n" << p.dVda << std::endl;
			s << "dVdg:\n" << p.dVdg << std::endl;
			s << "dPda:\n" << p.dPda << std::endl;
			s << "dPdg:\n" << p.dPdg << std::endl;
			s << "cov:\n" << p.Cov<< std::endl;
			return s;
		}

		void reset() {
			dT = 0;

			dR.setIdentity();
			dV.setZero();
			dP.setZero();
			Cov.setZero();

			dRdg.setZero();
			dVdg.setZero();
			dVda.setZero();
			dPdg.setZero();
			dPda.setZero();
		}
	};
}