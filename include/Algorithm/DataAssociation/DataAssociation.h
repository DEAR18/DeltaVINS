#pragma once
#include "Algorithm/solver/SquareRootEKFSolver.h"
#include "dataStructure/vioStructures.h"

namespace DeltaVins
{
	namespace DataAssociation
	{

		void initDataAssociation(SquareRootEKFSolver* solver);
		int removeOutlierBy2PointRansac(Matrix3f& dR, std::list<TrackedFeature::Ptr>& vTrackedFeatures);

		void doDataAssociation(std::list<TrackedFeature::Ptr>& vTrackedFeatures,
		        bool bStatic);
		void DrawPointsAfterUpdates(std::vector<PointState*>& m_PointStates);
		void DrawPointsBeforeUpdates(std::vector<PointState*>& m_PointStates);
		void clear();

		
	}
	
}
