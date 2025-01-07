#pragma once
#include "dataStructure/filterStates.h"
#include "dataStructure/vioStructures.h"

#include <Algorithm/VIO_Constexprs.h>

namespace DeltaVins {
	
    struct ImuPreintergration;
    struct PointState;
    struct CamState;


    struct StaticMatrix
    {
    	
#if USE_GIVENS_MARGIN
        MatrixMfR m_infoFactorMatrixToMarginal;
#else
        MatrixXf m_infoFactorMatrixToMarginal;
#endif

#if USE_STATIC_STACK
        MatrixOfR m_StackedMatrix;
#else
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> m_StackedMatrix;
#endif

        MatrixMf m_infoFactorMatrix;//Upper Triangle Matrix

#if USE_KEYFRAME
        MatrixMf m_infoFactorMatrixAfterMarigin;
#endif

        VectorMf m_residual;

    };

	

    class SquareRootEKFSolver {
    public:

        friend class VIOAlgorithm;
    	
        SquareRootEKFSolver();


        void init(CamState* pCamState, Vector3f* vel
#if USE_PLANE_PRIOR
            , Vector3f* planeCoeff,
            Vector3f n
#endif
            , bool* bStatic
        );
        void addCamState(CamState *state);

        void propagate(const ImuPreintergration *pImuTerm);

        void propagateStatic(const ImuPreintergration *pImuTerm);

        void propagateNew(const ImuPreintergration *pImuTerm);

        void marginalize();

        void marginalizeNew();

        void marginalizeStatic();
    	
        void marginalizeGivens();

        bool MahalanobisTest(PointState *state);

        int computeJacobians(TrackedFeature* track);


        void addMsckfPoint(PointState *state);

        void addSlamPoint(PointState* state);

        void addVelocityConstraint(int nRows);

        int _addPlaneContraint();
        int stackInformationFactorMatrix();

        void solveAndUpdateStates();

        int _addNewSlamPointConstraint();

        int addSlamPointConstraint();

    	
    private:

        void _updateByGivensRotations(int row, int col);

        bool _detectPureRotation();

        int _addPositionContraint(int nRows);

#if USE_NEW_MOVED_PIXEL

        void _computeDeltaR();

#endif

#if USE_KEYFRAME


#endif

#ifdef PLATFORM_ARM
        void _updateByGivensRotationsNeon();

#endif
        void _marginByGivensRotation();


        //MatrixMf m_infoFactorInverseMatrix;

#if USE_PLANE_PRIOR

        Vector3f *m_planeCoeff;
        MatrixXf  m_PlaneH;
        Vector3f m_n;
#endif

#if USE_GIVENS_MARGIN
        MatrixMfR m_infoFactorMatrixToMarginal;
#else
        MatrixXf m_infoFactorMatrixToMarginal;
#endif

#if USE_STATIC_STACK
        MatrixOfR m_StackedMatrix;
#if REMOVE_RESIDUAL_STACK
        VectorOf m_obsResidual;
#endif

        int m_nStackRows =0;
#else
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> m_StackedMatrix;
#endif

        MatrixMf m_infoFactorMatrix;//Upper Triangle Matrix

#if USE_KEYFRAME
        MatrixMf m_infoFactorMatrixAfterMarigin;
#endif
        VectorMf m_residual;

    	

        int CURRENT_DIM = 0;
        std::vector<CamState *> m_vCamStates;
        std::vector<PointState *> m_vMsckfPoints;
#if USE_KEYFRAME
        std::vector<PointState*> m_vSlamPoint;
        std::vector<PointState*> m_vNewSlamPoint;
#endif
        Vector3f *m_pVel;
        bool* m_bStatic;
        CamState *m_pNewState = nullptr;
        CamState *m_pLastState = nullptr;
    };
}
