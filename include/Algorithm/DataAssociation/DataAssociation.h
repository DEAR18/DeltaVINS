#pragma once
#include "Algorithm/solver/SquareRootEKFSolver.h"
#include "dataStructure/vioStructures.h"

namespace DeltaVins {
namespace DataAssociation {

void InitDataAssociation(SquareRootEKFSolver* solver);
int RemoveOutlierBy2PointRansac(Matrix3f& dR,
                                std::list<Landmark::Ptr>& trackedFeatures,
                                int sensor_id, int cam_id);

void DoDataAssociation(std::list<Landmark::Ptr>& trackedFeatures, bool bstatic);
void DrawPointsAfterUpdates(std::vector<PointState*>& pointStates,
                            int cam_id = 0);
void DrawPointsBeforeUpdates(std::vector<PointState*>& pointStates,
                             int cam_id = 0);
void Clear();

}  // namespace DataAssociation

}  // namespace DeltaVins
