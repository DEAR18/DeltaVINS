#pragma once
#include <Algorithm/VIO_Constexprs.h>

#include "dataStructure/filterStates.h"
#include "dataStructure/vioStructures.h"

namespace DeltaVins {

struct ImuPreintergration;
struct PointState;
struct CamState;

class SquareRootEKFSolver {
   public:
    friend class VIOAlgorithm;

    SquareRootEKFSolver();

    void Init(CamState *pCamState, Vector3f *vel
#if USE_PLANE_PRIOR
              ,
              Vector3f *planeCoeff, Vector3f n
#endif
              ,
              bool *static_);
    void AddCamState(CamState *state);

    void Propagate(const ImuPreintergration *pImuTerm);

    void PropagateStatic(const ImuPreintergration *pImuTerm);

    void PropagateNew(const ImuPreintergration *pImuTerm);

    void Marginalize();

    void MarginalizeNew();

    void MarginalizeStatic();

    void MarginalizeGivens();

    bool MahalanobisTest(PointState *state);

    int ComputeJacobians(TrackedFeature *track);

    void AddMsckfPoint(PointState *state);

    void AddSlamPoint(PointState *state);

    void AddVelocityConstraint(int nRows);

    int _AddPlaneContraint();
    int StackInformationFactorMatrix();

    void SolveAndUpdateStates();

    int _AddNewSlamPointConstraint();

    int AddSlamPointConstraint();

   private:
    void _UpdateByGivensRotations(int row, int col);

    bool _DetectPureRotation();

    int _AddPositionContraint(int nRows);

#if USE_NEW_MOVED_PIXEL

    void _computeDeltaR();

#endif

#if USE_KEYFRAME

#endif

#ifdef PLATFORM_ARM
    void _updateByGivensRotationsNeon();

#endif
    void _MarginByGivensRotation();

    // MatrixMf m_infoFactorInverseMatrix;

#if USE_PLANE_PRIOR

    Vector3f *m_planeCoeff;
    MatrixXf m_PlaneH;
    Vector3f m_n;
#endif

#if USE_GIVENS_MARGIN
    MatrixMfR info_factor_matrix_to_marginal_;
#else
    MatrixXf info_factor_matrix_to_marginal_;
#endif

#if USE_STATIC_STACK
    MatrixOfR stacked_matrix_;
#if REMOVE_RESIDUAL_STACK
    VectorOf obs_residual_;
#endif

    int stacked_rows_ = 0;
#else
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
        stacked_matrix_;
#endif

    MatrixMf info_factor_matrix_;  // Upper Triangle Matrix

#if USE_KEYFRAME
    MatrixMf info_factor_matrix_after_mariginal_;
#endif
    VectorMf residual_;

    int CURRENT_DIM = 0;
    std::vector<CamState *> cam_states_;
    std::vector<PointState *> msckf_points_;
#if USE_KEYFRAME
    std::vector<PointState *> slam_point_;
    std::vector<PointState *> new_slam_point_;
#endif
    Vector3f *vel_;
    bool *static_;
    CamState *new_state_ = nullptr;
    CamState *last_state_ = nullptr;
};
}  // namespace DeltaVins
