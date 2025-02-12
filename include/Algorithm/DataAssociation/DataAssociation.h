#pragma once
#include "Algorithm/solver/SquareRootEKFSolver.h"
#include "dataStructure/vioStructures.h"

namespace DeltaVins {
namespace DataAssociation {

void InitDataAssociation(SquareRootEKFSolver* solver);
int RemoveOutlierBy2PointRansac(Matrix3f& dR,
                                std::list<TrackedFeature::Ptr>& trackedFeatures,
                                int sensor_id);

void DoDataAssociation(std::list<TrackedFeature::Ptr>& trackedFeatures,
                       bool bstatic);
void DrawPointsAfterUpdates(std::vector<PointState*>& pointStates);
void DrawPointsBeforeUpdates(std::vector<PointState*>& pointStates);
void Clear();

}  // namespace DataAssociation

}  // namespace DeltaVins
