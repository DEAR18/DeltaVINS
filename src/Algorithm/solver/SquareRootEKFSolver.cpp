#include "Algorithm/solver/SquareRootEKFSolver.h"

#include <utils/TickTock.h>

#include <sophus/so3.hpp>

#include "Algorithm/IMU/ImuPreintergration.h"
#include "Algorithm/vision/camModel/camModel.h"
#include "IO/dataBuffer/imuBuffer.h"
#include "precompile.h"
#include "utils/utils.h"

#define GRAVITY 9.81f

namespace DeltaVins {
SquareRootEKFSolver::SquareRootEKFSolver() {
    // m_infoFactorMatrix.resize(MAX_MATRIX_SIZE,MAX_MATRIX_SIZE);
    // m_infoFactorMatrixToMarginal.resize(MAX_MATRIX_SIZE,MAX_MATRIX_SIZE);
    // m_residual.resize(MAX_MATRIX_SIZE,1);
    //
    //
}

void SquareRootEKFSolver::Init(CamState* pCamState, Vector3f* vel
#if USE_PLANE_PRIOR
                               ,
                               Vector3f* planeCoeff, Vector3f n
#endif
                               ,
                               bool* bStatic) {

#if USE_PLANE_PRIOR
    VectorXf p(NEW_STATE_DIM + PLANE_DIM);

    switch (Config::DataSourceType) {
        case DataSrcEuroc:
            p << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2, 1e-3,
                1e-3, 1e-3, 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2;
            break;
        default:
            p << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2, 1e-3,
                1e-3, 1e-3, 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2;
            break;
    }

    CURRENT_DIM = NEW_STATE_DIM + PLANE_DIM;
#else
    VectorXf p(NEW_STATE_DIM);

    switch (Config::DataSourceType) {
        case DataSrcEuroc:
            p << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2, 1e-3, 1e-3, 1e-3, 1e-4,
                1e-4, 1e-4, 1e-2, 1e-2, 1e-2;
            break;
        default:
            p << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2, 1e-3, 1e-3, 1e-3, 1e-4,
                1e-4, 1e-4, 1e-2, 1e-2, 1e-2;
            break;
    }

    CURRENT_DIM = NEW_STATE_DIM;
#endif

    m_pVel = vel;
    m_bStatic = bStatic;
#if USE_PLANE_PRIOR

    m_planeCoeff = planeCoeff;
    m_n = n;
#endif
    m_infoFactorMatrix.topLeftCorner(CURRENT_DIM, CURRENT_DIM) =
        p.cwiseSqrt().cwiseInverse().asDiagonal();

    m_pVel->setZero();
    AddCamState(pCamState);
    m_vCamStates.push_back(pCamState);
}

void SquareRootEKFSolver::AddCamState(CamState* state) {
    m_pLastState = m_pNewState;
    m_pNewState = state;
}
void SquareRootEKFSolver::PropagateStatic(const ImuPreintergration* pImuTerm) {
#if USE_Z_AXIS
    static Vector3f gravity(0, 0, GRAVITY);
#else
    static Vector3f gravity(0, GRAVITY, 0);
#endif
    Matrix3f R0 = m_pLastState->m_Rwi;

    float dt = pImuTerm->dT * 1e-9;

    // propagate states
    m_pNewState->m_Pwi =
        m_pLastState->m_Pwi +
        (R0 * pImuTerm->dP + *m_pVel * dt + gravity * (0.5f * dt * dt));
    m_pNewState->m_Pw_FEJ = m_pNewState->m_Pwi;
    *m_pVel += R0 * pImuTerm->dV + gravity * dt;
    m_pNewState->m_Rwi = R0 * pImuTerm->dR;

    // Make state transition matrix F
    static Eigen::Matrix<float, NEW_STATE_DIM, NEW_STATE_DIM>
        StateTransitionMatrix;
    StateTransitionMatrix.setIdentity();
    StateTransitionMatrix.block<3, 3>(0, 0) = pImuTerm->dR.transpose();
    StateTransitionMatrix.block<3, 3>(0, 6) = pImuTerm->dRdg;
    StateTransitionMatrix.block<3, 3>(3, 0) = -R0 * crossMat(pImuTerm->dP);
    StateTransitionMatrix.block<3, 3>(3, 6) = R0 * pImuTerm->dPdg;
    StateTransitionMatrix.block<3, 3>(3, 9) = Matrix3f::Identity() * dt;
    StateTransitionMatrix.block<3, 3>(9, 0) = -R0 * crossMat(pImuTerm->dV);
    StateTransitionMatrix.block<3, 3>(9, 6) = R0 * pImuTerm->dVdg;
#if !DISABLE_ACC_BIAS
    StateTransitionMatrix.block<3, 3>(9, 12) = R0 * pImuTerm->dVda;
    StateTransitionMatrix.block<3, 3>(3, 12) = R0 * pImuTerm->dPda;
#endif

#if USE_SQ_NOISE

    // make noise transition matrix
    static Eigen::Matrix<float, 9, 9> NoiseTransitionMatrix;
    NoiseTransitionMatrix.setZero();
    NoiseTransitionMatrix.block<3, 3>(0, 0).setIdentity();
    NoiseTransitionMatrix.block<3, 3>(3, 6) = R0;
    NoiseTransitionMatrix.block<3, 3>(6, 3) = R0;

    Eigen::LLT<Matrix9f> chol(pImuTerm->Cov);
    Matrix9f covFactor;
    covFactor.setIdentity();
    chol.matrixU().solveInPlace(covFactor);
    covFactor = NoiseTransitionMatrix * covFactor;
    static Eigen::Matrix<float, NEW_STATE_DIM, NEW_STATE_DIM> NoiseFactor;

    NoiseFactor.setZero();
    NoiseFactor.topLeftCorner<6, 6>() = covFactor.topLeftCorner<6, 6>();
    NoiseFactor.block<6, 3>(0, 9) = covFactor.block<6, 3>(0, 6);
    NoiseFactor.block<3, 6>(9, 0) = covFactor.block<3, 6>(9, 0);
    NoiseFactor.block<3, 3>(9, 9) = covFactor.block<3, 3>(6, 6);
    NoiseFactor.diagonal().segment<3>(6) =
        Vector3f::Ones() * (1.f / (sqrt(Config::GyroBiasNoise2) * dt));
    NoiseFactor.diagonal().segment<3>(12) =
        Vector3f::Ones() * (1.f / (sqrt(Config::AccBiasNoise2) * dt));

#else
    // make noise transition matrix
    static Eigen::Matrix<float, NEW_STATE_DIM, 9> NoiseTransitionMatrix;
    NoiseTransitionMatrix.setZero();
    NoiseTransitionMatrix.block<3, 3>(0, 0).setIdentity();
    NoiseTransitionMatrix.block<3, 3>(3, 6) = R0;
    NoiseTransitionMatrix.block<3, 3>(9, 3) = R0;

    // Make Noise Covariance Matrix Q
    static Eigen::Matrix<float, NEW_STATE_DIM, NEW_STATE_DIM> NoiseCov;
    NoiseCov = NoiseTransitionMatrix * pImuTerm->Cov *
               NoiseTransitionMatrix.transpose();
    NoiseCov.block<3, 3>(6, 6) =
        Matrix3f::Identity() * (Config::GyroBiasNoise2 * dt * dt);
#if !DISABLE_ACC_BIAS
    NoiseCov.block<3, 3>(12, 12) =
        Matrix3f::Identity() * (Config::AccBiasNoise2 * dt * dt);
#endif

    static Eigen::Matrix<float, NEW_STATE_DIM, NEW_STATE_DIM> NoiseFactor;
    NoiseFactor.setIdentity();
    Eigen::LLT<MatrixXf> chol(NoiseCov);
    chol.matrixU().solveInPlace(NoiseFactor);

#endif

    // Make information factor matrix
    int OLD_DIM = CURRENT_DIM;
    CURRENT_DIM += NEW_STATE_DIM;

#if USE_PLANE_PRIOR
    int IMUIdx = PLANE_DIM;
#else
    int IMUIdx = 0;
#endif
    m_infoFactorMatrix.block(0, OLD_DIM, OLD_DIM, NEW_STATE_DIM).setZero();
    m_infoFactorMatrix.block(OLD_DIM, 0, NEW_STATE_DIM, OLD_DIM).setZero();
    MatrixXf R = NoiseFactor * StateTransitionMatrix;
    m_infoFactorMatrix.block<NEW_STATE_DIM, IMU_STATE_DIM>(OLD_DIM, IMUIdx) =
        R.rightCols<IMU_STATE_DIM>();
    m_infoFactorMatrix.block<NEW_STATE_DIM, CAM_STATE_DIM>(
        OLD_DIM, OLD_DIM - CAM_STATE_DIM) = R.leftCols<CAM_STATE_DIM>();
    // m_infoFactorMatrix.block<NEW_STATE_DIM, NEW_STATE_DIM>(N, N -
    // NEW_STATE_DIM) = NoiseFactor * StateTransitionMatrix;
    m_infoFactorMatrix.block<NEW_STATE_DIM, NEW_STATE_DIM>(OLD_DIM, OLD_DIM) =
        -NoiseFactor;

    // make residual vector
    m_residual.segment(0, CURRENT_DIM).setZero();

#if USE_VELOCITY_DETECT_ROTATION
    m_pNewState->m_vel = *m_pVel;
#endif
}
#if 0
	void SquareRootEKFSolver::Propagate(const ImuPreintergration* pImuTerm)
    {

        static Vector3f gravity(0, GRAVITY, 0);
        Matrix3f R0 = m_pLastState->m_Rwi;

        float dt = pImuTerm->dT * 1e-9;

    	// propagate states
        m_pNewState->m_Pwi = m_pLastState->m_Pwi + (R0 * pImuTerm->dP + *m_pVel * dt + gravity * (0.5f * dt * dt));
        m_pNewState->m_Pw_FEJ = m_pNewState->m_Pwi;
        *m_pVel += R0 * pImuTerm->dV + gravity * dt;
        m_pNewState->m_Rwi = R0 * pImuTerm->dR;

        //Make state transition matrix F
        Eigen::Matrix<float,NEW_STATE_DIM,NEW_STATE_DIM> StateTransitionMatrix;
        StateTransitionMatrix.setIdentity();
        StateTransitionMatrix.block<3, 3>(0, 0) = pImuTerm->dR.transpose();
        StateTransitionMatrix.block<3, 3>(0, 6) = pImuTerm->dRdg;
        StateTransitionMatrix.block<3, 3>(3, 0) = -R0 * crossMat(pImuTerm->dP);
        StateTransitionMatrix.block<3, 3>(3, 6) = R0 * pImuTerm->dPdg;
        StateTransitionMatrix.block<3, 3>(3, 9) = Matrix3f::Identity() * dt;
        StateTransitionMatrix.block<3, 3>(9, 0) = -R0 * crossMat(pImuTerm->dV);
        StateTransitionMatrix.block<3, 3>(9, 6) = R0 * pImuTerm->dVdg;
#if !DISABLE_ACC_BIAS
        StateTransitionMatrix.block<3, 3>(9, 12) = R0 * pImuTerm->dVda;
        StateTransitionMatrix.block<3, 3>(3, 12) = R0 * pImuTerm->dPda;
#endif

        //make noise transition matrix
        Eigen::Matrix<float,NEW_STATE_DIM,9> NoiseTransitionMatrix;
        NoiseTransitionMatrix.setZero();
        NoiseTransitionMatrix.block<3, 3>(0, 0).setIdentity();
        NoiseTransitionMatrix.block<3, 3>(3, 6) = R0;
        NoiseTransitionMatrix.block<3, 3>(9, 3) = R0;

        //Make Noise Covariance Matrix Q
        Eigen::Matrix<float,NEW_STATE_DIM,NEW_STATE_DIM> NoiseCov = NoiseTransitionMatrix * pImuTerm->Cov * NoiseTransitionMatrix.transpose();
        NoiseCov.block<3, 3>(6, 6) = Matrix3f::Identity() * (Config::GyroBiasNoise2 * dt * dt);
#if !DISABLE_ACC_BIAS
        NoiseCov.block<3, 3>(12, 12) = Matrix3f::Identity() * (Config::AccBiasNoise2 * dt * dt);
#endif

        Eigen::Matrix<float,NEW_STATE_DIM,NEW_STATE_DIM> NoiseFactor;
        NoiseFactor.setIdentity();
        Eigen::LLT<MatrixXf> chol(NoiseCov);
        chol.matrixU().solveInPlace(NoiseFactor);

        //Make information factor matrix
        int N = m_infoFactorMatrix.rows();
        m_infoFactorMatrix.conservativeResize(N + NEW_STATE_DIM, N + NEW_STATE_DIM);

    	MatrixXf A = NoiseFactor * StateTransitionMatrix;

        m_infoFactorMatrix.topRightCorner(N, NEW_STATE_DIM).setZero();
        m_infoFactorMatrix.bottomLeftCorner(NEW_STATE_DIM, N).setZero();
        m_infoFactorMatrix.block<NEW_STATE_DIM, NEW_STATE_DIM>(N, N - NEW_STATE_DIM) = A;
        m_infoFactorMatrix.bottomRightCorner<NEW_STATE_DIM, NEW_STATE_DIM>() = -NoiseFactor;



        // make residual vector
        m_residual = VectorXf::Zero(m_infoFactorMatrix.rows());

    }

    void SquareRootEKFSolver::PropagateNew(const ImuPreintergration* pImuTerm)
    {
        static Vector3f gravity(0, GRAVITY, 0);
        Matrix3f R0 = m_pLastState->m_Rwi;

        float dt = pImuTerm->dT * 1e-9;

    	// propagate states
        m_pNewState->m_Pwi = m_pLastState->m_Pwi + (R0 * pImuTerm->dP + *m_pVel * dt + gravity * (0.5f * dt * dt));
        m_pNewState->m_Pw_FEJ = m_pNewState->m_Pwi;
        *m_pVel += R0 * pImuTerm->dV + gravity * dt;
        m_pNewState->m_Rwi = R0 * pImuTerm->dR;

        //Make state transition matrix F
        Eigen::Matrix<float, NEW_STATE_DIM, NEW_STATE_DIM> StateTransitionMatrix;
        StateTransitionMatrix.setIdentity();
        StateTransitionMatrix.block<3, 3>(0, 0) = pImuTerm->dR.transpose();
        StateTransitionMatrix.block<3, 3>(0, 6) = pImuTerm->dRdg;
        StateTransitionMatrix.block<3, 3>(3, 0) = -R0 * crossMat(pImuTerm->dP);
        StateTransitionMatrix.block<3, 3>(3, 6) = R0 * pImuTerm->dPdg;
        StateTransitionMatrix.block<3, 3>(3, 9) = Matrix3f::Identity() * dt;
        StateTransitionMatrix.block<3, 3>(3, 12) = R0 * pImuTerm->dPda;
        StateTransitionMatrix.block<3, 3>(9, 0) = -R0 * crossMat(pImuTerm->dV);
        StateTransitionMatrix.block<3, 3>(9, 6) = R0 * pImuTerm->dVdg;
        StateTransitionMatrix.block<3, 3>(9, 12) = R0 * pImuTerm->dVda;

        //make noise transition matrix
        Eigen::Matrix<float, NEW_STATE_DIM, 9> NoiseTransitionMatrix;
        NoiseTransitionMatrix.setZero();
        NoiseTransitionMatrix.block<3, 3>(0, 0).setIdentity();
        NoiseTransitionMatrix.block<3, 3>(3, 6) = R0;
        NoiseTransitionMatrix.block<3, 3>(9, 3) = R0;

        //Make Noise Covariance Matrix Q
        Eigen::Matrix<float, NEW_STATE_DIM, NEW_STATE_DIM> NoiseCov = NoiseTransitionMatrix * pImuTerm->Cov * NoiseTransitionMatrix.transpose();
        NoiseCov.block<3, 3>(6, 6) = Matrix3f::Identity() * (Config::GyroBiasNoise2 * dt * dt);
        NoiseCov.block<3, 3>(12, 12) = Matrix3f::Identity() * (Config::AccBiasNoise2 * dt * dt);

        Eigen::Matrix<float, NEW_STATE_DIM, NEW_STATE_DIM> NoiseFactor;
        NoiseFactor.setIdentity();
        Eigen::LLT<MatrixXf> chol(NoiseCov);
        chol.matrixU().solveInPlace(NoiseFactor);

        //Make information factor matrix
        int N = m_infoFactorMatrix.rows();
        m_infoFactorMatrix.conservativeResize(N + NEW_STATE_DIM, N + NEW_STATE_DIM);

        m_infoFactorMatrix.topRightCorner(N, NEW_STATE_DIM).setZero();
        m_infoFactorMatrix.bottomLeftCorner(NEW_STATE_DIM, N).setZero();
        MatrixXf R = NoiseFactor * StateTransitionMatrix;
        m_infoFactorMatrix.block<NEW_STATE_DIM,IMU_STATE_DIM>(N, 0) = R.rightCols<IMU_STATE_DIM>();
        m_infoFactorMatrix.block<NEW_STATE_DIM, CAM_STATE_DIM>(N, N-CAM_STATE_DIM) = R.leftCols<CAM_STATE_DIM>();
        //m_infoFactorMatrix.block<NEW_STATE_DIM, NEW_STATE_DIM>(N, N - NEW_STATE_DIM) = NoiseFactor * StateTransitionMatrix;
        m_infoFactorMatrix.bottomRightCorner<NEW_STATE_DIM, NEW_STATE_DIM>() = -NoiseFactor;

        // make residual vector
        m_residual = VectorXf::Zero(m_infoFactorMatrix.rows());


    }

    void SquareRootEKFSolver::marginalize()
    {
        int iDim = 0;
        std::vector<int> v_MarginDIM,v_RemainDIM;
        std::vector<CamState*> v_CamStateNew;
        for (int i=0,n = m_vCamStates.size();i<n;++i)
        {
            auto state = m_vCamStates[i];
            if (state->m_bToMargin)
                for (int i = 0; i < CAM_STATE_DIM; i++)
                    v_MarginDIM.push_back(iDim++);
            else
            {
                for (int i = 0; i < CAM_STATE_DIM; i++)
                    v_RemainDIM.push_back(iDim++);
                v_CamStateNew.push_back(state);
            }
        }
        m_vCamStates = v_CamStateNew;
        m_vCamStates.push_back(m_pNewState);
        for (int i=0,n=m_vCamStates.size();i<n;++i)
        {
            m_vCamStates[i]->m_idx = i;
        }

        for (int i = 0; i < IMU_STATE_DIM; i++)
        {
            v_MarginDIM.push_back(iDim++);
        }
        for (int i = 0; i < NEW_STATE_DIM; i++)
        {
            v_RemainDIM.push_back(iDim++);
        }

        int nCols = m_infoFactorMatrix.cols();

        //rearrange new information factor matrix
        MatrixXf NewInfoFactor(nCols, nCols);
        int index = 0;
        for (auto idx : v_MarginDIM)
        {
            NewInfoFactor.col(index++) = m_infoFactorMatrix.col(idx);
        }
        for (auto idx : v_RemainDIM)
        {
            NewInfoFactor.col(index++) = m_infoFactorMatrix.col(idx);
        }

        //use qr to marginalize states
        Eigen::HouseholderQR<MatrixXf> qr(NewInfoFactor);
        int nRemain = v_RemainDIM.size();

        m_infoFactorMatrix = qr.matrixQR().bottomRightCorner(nRemain, nRemain).triangularView<Eigen::Upper>();

        m_infoFactorInverseMatrix = MatrixXf::Identity(nRemain, nRemain);
        m_infoFactorMatrix.triangularView<Eigen::Upper>().solveInPlace(m_infoFactorInverseMatrix);

        m_residual.resize(m_infoFactorMatrix.rows(), 1);
        m_residual.setZero();



    }

    void SquareRootEKFSolver::marginalizeNew()
    {
        int iDim = 0;
        std::vector<int> v_MarginDIM, v_RemainDIM;
        std::vector<CamState*> v_CamStateNew;

        int N = m_infoFactorMatrix.cols();
        for (int i=0;i<IMU_STATE_DIM;++i)
        {
            v_MarginDIM.push_back(i);
            v_RemainDIM.push_back(N - IMU_STATE_DIM + i);
        }
        iDim = IMU_STATE_DIM;
        for (int i = 0, n = m_vCamStates.size(); i < n; ++i)
        {
            auto state = m_vCamStates[i];
            if (state->m_bToMargin)
                for (int i = 0; i < CAM_STATE_DIM; i++)
                    v_MarginDIM.push_back(iDim++);
            else
            {
                for (int i = 0; i < CAM_STATE_DIM; i++)
                    v_RemainDIM.push_back(iDim++);
                v_CamStateNew.push_back(state);
            }
        }
        m_vCamStates = v_CamStateNew;
        m_vCamStates.push_back(m_pNewState);
        for (int i = 0, n = m_vCamStates.size(); i < n; ++i)
        {
            m_vCamStates[i]->m_idx = i;
        }

        for (int i = 0; i < CAM_STATE_DIM; i++)
        {
            v_RemainDIM.push_back(iDim++);
        }

        int nCols = m_infoFactorMatrix.cols();

        //rearrange new information factor matrix
        MatrixXf NewInfoFactor(nCols, nCols);
        int index = 0;
        for (auto idx : v_MarginDIM)
        {
            NewInfoFactor.col(index++) = m_infoFactorMatrix.col(idx);
        }
        for (auto idx : v_RemainDIM)
        {
            NewInfoFactor.col(index++) = m_infoFactorMatrix.col(idx);
        }





        //use qr to marginalize states
        Eigen::HouseholderQR<MatrixXf> qr(NewInfoFactor);
        int nRemain = v_RemainDIM.size();

        m_infoFactorMatrix = qr.matrixQR().bottomRightCorner(nRemain, nRemain).triangularView<Eigen::Upper>();
    	

        m_infoFactorInverseMatrix = MatrixXf::Identity(nRemain, nRemain);
        m_infoFactorMatrix.triangularView<Eigen::Upper>().solveInPlace(m_infoFactorInverseMatrix);

        m_residual.resize(m_infoFactorMatrix.rows(), 1);
        m_residual.setZero();

    }

#endif

bool SquareRootEKFSolver::MahalanobisTest(PointState* state) {
    VectorXf z = state->H.rightCols<1>();
    int nExceptPoint = state->H.cols() - 1;
    int nObs = z.rows();
#if USE_NAIVE_ML_DATAASSOCIATION
    float phi;
#if USE_KEYFRAME
    if (state->bSlamPoint) {
        Matrix2f S;
        Matrix2f R = MatrixXf::Identity(2, 2) * Config::ImageNoise2 * 2;
        MatrixXf E;
        E.resize(CURRENT_DIM, 9);
#if USE_PLANE_PRIOR
        int iLeft = IMU_STATE_DIM + PLANE_DIM;
#else
        int iLeft = IMU_STATE_DIM;
#endif
        E.leftCols<3>() = m_infoFactorMatrixAfterMarigin.block(
            0, state->m_idx * 3 + iLeft, CURRENT_DIM, 3);
        E.rightCols<6>() = m_infoFactorMatrixAfterMarigin.block(
            0,
            state->host->m_vVisualObs.back().m_linkFrame->state->m_idx *
                    CAM_STATE_DIM +
                iLeft + 3 * m_vSlamPoint.size(),
            CURRENT_DIM, 6);
        Matrix9f F = E.transpose() * E;
        S = state->H.leftCols(9) * F.inverse() *
            state->H.leftCols(9).transpose();
        S.noalias() += R;

        phi = z.transpose() * S.inverse() * z;
    } else {
#endif
        phi = z.dot(z) / (2 * Config::ImageNoise2);
#if USE_KEYFRAME
    }
#endif
#else
    MatrixXf S = MatrixXf::Zero(nObs, nObs);
    MatrixXf R = MatrixXf::Identity(nObs, nObs) * Config::ImageNoise2 * 2;

    MatrixXf B =
        state->H.leftCols(nExceptPoint) *
        m_infoFactorInverseMatrix.topLeftCorner(nExceptPoint, nExceptPoint)
            .triangularView<Eigen::Upper>();
    nn S.noalias() += R;

    float phi = z.transpose() * S.llt().solve(z);
#endif

    assert(nObs < 80);
#if OUTPUT_DEBUG_INFO
    if (phi < chi2LUT[nObs])
        printf("#### MahalanobisTest Success\n");
    else
        printf("#### MahalanobisTest Fail\n");

#endif
    return phi < chi2LUT[nObs];
}

void rowMajorMatrixQRByGivensInMsckf(MatrixHfR& H, int row, int col) {
    int nrows = row;
    int nCols = col;
    assert(H.IsRowMajor);
    for (size_t j = 0; j < 3; ++j) {
        for (size_t i = H.rows() - 1; i > j; i--) {
            float* pI = H.row(i).data();
            float* pJ = H.row(i - 1).data();
            float c, s;
            float alpha = pJ[j];
            float beta = pI[j];
            if (fabs(beta) < FLT_EPSILON) {
                continue;
            } else if (fabs(alpha) < FLT_EPSILON) {
                for (int k = j; k < nCols; ++k) {
                    float x = -pI[k];
                    float y = pJ[k];

                    pJ[k] = x;
                    pI[k] = y;
                }
                continue;
            } else if (fabs(beta) > fabs(alpha)) {
                s = 1 / sqrt(1 + pow(alpha / beta, 2));
                c = -alpha / beta * s;
            } else {
                c = 1 / sqrt(1 + pow(beta / alpha, 2));
                s = -beta / alpha * c;
            }
            for (int k = j; k < nCols; ++k) {
                float x = c * pJ[k] - s * pI[k];
                float y = s * pJ[k] + c * pI[k];

                pJ[k] = x;
                pI[k] = y;
            }
        }
    }
}

int SquareRootEKFSolver::computeJacobians(TrackedFeature* track) {
    // observation number
    int index = 0;

    static CamModel* camModel = CamModel::getCamModel();
    static Vector3f Tci = camModel->getTci();
    int nCams = m_vCamStates.size();

    const int camStateIdx = 3;

    const int residualIdx = 3 + CAM_STATE_DIM * nCams;

    int nObs = track->m_vVisualObs.size();

#if USE_KEYFRAME
    if (track->m_pState->bSlamPoint) {
        nObs = 1;
    }
#endif
    float huberThresh = 500.f;
    float cutOffThresh = 10.f;

    // Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> H;
    auto& H = track->m_pState->H;
    H.setZero(nObs * 2, 3 + (CAM_STATE_DIM * nCams) + 1);
    auto calcObsJac = [&](VisualObservation* ob) {
        int cam_id = ob->m_linkFrame->state->m_idx;

        Matrix3f Riw = ob->m_linkFrame->state->m_Rwi.transpose();
        Vector3f Pi =
            Riw * (track->m_pState->m_Pw - ob->m_linkFrame->state->m_Pwi);
        Vector3f Pi_FEJ = Riw * (track->m_pState->m_Pw_FEJ -
                                 ob->m_linkFrame->state->m_Pw_FEJ);

        Vector2f px = camModel->imuToImage(Pi);
        Vector2f r = ob->m_px - px;
        float reprojErr = r.norm();

        if (reprojErr > cutOffThresh) {
            track->reproject();
            track->drawObservationsAndReprojection(1);
            printf("%f %f ->%f %f\n", ob->m_px.x(), ob->m_px.y(), px.x(),
                   px.y());
            return false;
        }

        H.block<2, 1>(2 * index, residualIdx) = r;

        Matrix23f J23;
        camModel->camToImage(Pi_FEJ + Tci, J23);

        H.block<2, 3>(2 * index, camStateIdx + CAM_STATE_DIM * cam_id) =
            J23 * crossMat(Pi_FEJ);
#if USE_POSITION_DETECT_ROTATION
        if (!track->m_bInaccurateDepth)
#endif
            H.block<2, 3>(2 * index, camStateIdx + CAM_STATE_DIM * cam_id + 3) =
                -J23 * Riw;
        H.block<2, 3>(2 * index, 0) = J23 * Riw;

        return true;
    };

#if USE_KEYFRAME
    if (track->m_pState->bSlamPoint) {
        if (calcObsJac(&track->m_vVisualObs.back())) {
            index++;
        }
        if (index != nObs) {
            return 0;
        }
    } else {
#endif

        for (auto& ob : track->m_vVisualObs) {
            if (calcObsJac(&ob)) {
                index++;
            }
        }

#if USE_KEYFRAME
    }

#endif
    if (index != nObs) {
        if (index < 2) return 0;
        nObs = index;
        H.conservativeResize(2 * index, Eigen::NoChange);
    }

    return 2 * nObs;
}

#if USE_KEYFRAME

int SquareRootEKFSolver::_addNewSlamPointConstraint() {
    int nObs = 0;

    for (auto point : m_vSlamPoint) {
        int obs = computeJacobians(point->host);

        if (obs && MahalanobisTest(point))
            nObs += obs;
        else
            point->m_bToMargin = true;
    }

    if (m_vNewSlamPoint.empty()) {
        m_infoFactorMatrix.topLeftCorner(CURRENT_DIM, CURRENT_DIM) =
            m_infoFactorMatrixAfterMarigin.topLeftCorner(CURRENT_DIM,
                                                         CURRENT_DIM);
        return nObs;
    }

    int nNewSlamPointStates = m_vNewSlamPoint.size() * 3;

    int nOldSlamPointStates = m_vSlamPoint.size() * 3;
    int nCams = m_vCamStates.size();

#if USE_PLANE_PRIOR
    int nLeft = nOldSlamPointStates + IMU_STATE_DIM + PLANE_DIM;
#else
    int nLeft = nOldSlamPointStates + IMU_STATE_DIM;
#endif

    int nRight = nCams * CAM_STATE_DIM;

    int nNewCam = nLeft + nNewSlamPointStates;

    m_infoFactorMatrix.topLeftCorner(nLeft, nLeft) =
        m_infoFactorMatrixAfterMarigin.topLeftCorner(nLeft, nLeft);
    m_infoFactorMatrix.block(0, nNewCam, nLeft, nRight) =
        m_infoFactorMatrixAfterMarigin.block(0, nLeft, nLeft, nRight);
    m_infoFactorMatrix.block(nNewCam, 0, nRight, nLeft) =
        m_infoFactorMatrixAfterMarigin.block(nLeft, 0, nRight, nLeft);
    m_infoFactorMatrix.block(nNewCam, nNewCam, nRight, nRight) =
        m_infoFactorMatrixAfterMarigin.block(nLeft, nLeft, nRight, nRight);

    m_infoFactorMatrix
        .block(0, nLeft, CURRENT_DIM + nNewSlamPointStates, nNewSlamPointStates)
        .setZero();
    m_infoFactorMatrix
        .block(nLeft, 0, nNewSlamPointStates, CURRENT_DIM + nNewSlamPointStates)
        .setZero();

    CURRENT_DIM += nNewSlamPointStates;

    for (auto state : m_vNewSlamPoint) {
        state->host->removeUselessObservationForSlamPoint();
    }

    for (auto& state : m_vNewSlamPoint) {
        nObs += state->H.rows();
    }

    m_vSlamPoint.insert(m_vSlamPoint.end(), m_vNewSlamPoint.begin(),
                        m_vNewSlamPoint.end());

    for (int i = 0, n = m_vSlamPoint.size(); i < n; ++i) {
        m_vSlamPoint[i]->m_idx = i;
    }

    m_vNewSlamPoint.clear();

    return nObs;
}

int SquareRootEKFSolver::addSlamPointConstraint() {
    int nObs = 0;
    return nObs;
}

#endif

void SquareRootEKFSolver::addMsckfPoint(PointState* state) {
    constexpr int MAX_DIM =
        3 + (CAM_STATE_DIM * MAX_WINDOW_SIZE + IMU_STATE_DIM + 1);
    static MatrixHfR H;
    // do null space trick to get pose constraint
    int col = state->H.cols();
    int row = state->H.rows();
    H.topLeftCorner(row, col) = state->H;

    rowMajorMatrixQRByGivensInMsckf(H, row, col);
    state->H = H.block(3, 3, row - 3, col - 3);

    m_vMsckfPoints.push_back(state);
}

#if USE_KEYFRAME
void SquareRootEKFSolver::addSlamPoint(PointState* state) {
    m_vNewSlamPoint.push_back(state);
    state->bSlamPoint = true;
}
#endif

void SquareRootEKFSolver::addVelocityConstraint(int nRows) {
    static float invSigma = 1.0 / 1e-2;

    static float invRotSigma = 1.0 / 1e-5;
    static float invPosSigma = 1.0 / 1e-2;

#if DISABLE_ACC_BIAS
    int velIdx = nCols - 4;
#else
    int velIdx = 3;
#endif

#if USE_STATIC_STACK
    m_StackedMatrix.middleRows(nRows, 9).setZero();
#else
    m_StackedMatrix.conservativeResize(nRows + 9, Eigen::NoChange);
    m_StackedMatrix.bottomRows<9>().setZero();
#endif
    Matrix3f dR = m_pNewState->m_Rwi.transpose() * m_pLastState->m_Rwi;
    Vector3f dP = m_pNewState->m_Pwi - m_pLastState->m_Pwi;

    Eigen::AngleAxisf angleAxis;
    angleAxis.fromRotationMatrix(dR);
    Vector3f so3 = angleAxis.axis() * angleAxis.angle();
    Matrix3f crossSo3 = crossMat(so3);
    float absAngle = fabs(angleAxis.angle());
    Matrix3f Jr, Jl;
    if (absAngle < FLT_EPSILON) {
        Jr.setIdentity();
        Jl.setIdentity();
    } else {
        Jr = Eigen::Matrix3f::Identity() + 0.5 * crossSo3 +
             (pow(1.f / absAngle, 2) -
              (1 + cos(absAngle) / (2 * absAngle * sin(absAngle)))) *
                 crossSo3 * crossSo3;
        Jl = Jr.transpose();
    }
    m_StackedMatrix.block<3, 3>(nRows, CURRENT_DIM - 2 * CAM_STATE_DIM) =
        Jr * invRotSigma;
    m_StackedMatrix.block<3, 3>(nRows, CURRENT_DIM - CAM_STATE_DIM) =
        -Jl * invRotSigma;
#if REMOVE_RESIDUAL_STACK
    m_obsResidual.segment(nRows, 3) = -so3 * invRotSigma;
#else
    m_StackedMatrix.block<3, 1>(nRows, CURRENT_DIM) = -so3 * invRotSigma;
#endif
    m_StackedMatrix.block<3, 3>(nRows + 3,
                                CURRENT_DIM - 2 * CAM_STATE_DIM + 3) =
        -Matrix3f::Identity() * invPosSigma;
    m_StackedMatrix.block<3, 3>(nRows + 3, CURRENT_DIM - CAM_STATE_DIM + 3) =
        Matrix3f::Identity() * invPosSigma;
#if REMOVE_RESIDUAL_STACK
    m_obsResidual.segment(nRows + 3, 3) = -dP * invPosSigma;
#else
    m_StackedMatrix.block<3, 1>(nRows + 3, CURRENT_DIM) = -dP * invPosSigma;
#endif
    m_StackedMatrix.block<3, 3>(nRows + 6, velIdx) =
        Matrix3f::Identity() * invSigma;
#if REMOVE_RESIDUAL_STACK
    m_obsResidual.segment(nRows + 6, 3) = -*m_pVel * invSigma;
#else
    m_StackedMatrix.block<3, 1>(nRows + 6, CURRENT_DIM) = -*m_pVel * invSigma;
#endif
#if USE_STATIC_STACK
    m_nStackRows += 9;
#endif
}

#if USE_PLANE_PRIOR
int SquareRootEKFSolver::_addPlaneContraint() {
    static const float invStd = 1.0 / 1e-5;
    int nCams = m_vCamStates.size();
    m_PlaneH.setZero(nCams, nCams * 6 + 3 + 1);
    if (!Config::PlaneConstraint) return nCams;
    int residualIdx = nCams * 6 + 3;

#if USE_Z_AXIS
    Vector3f theta(m_planeCoeff->x(), m_planeCoeff->y(), 0);
#else
    Vector3f theta(m_planeCoeff->x(), 0, m_planeCoeff->y());
#endif
    float d = m_planeCoeff->z();

    Matrix3f R = Sophus::SO3f::exp(theta).matrix();

    float totalResidual = 0;

    int i = 0;
    for (auto camState : m_vCamStates) {
        auto& P = camState->m_Pwi;

        Vector3f dYdtheta = -P.transpose() * crossMat(m_n);
        Vector3f dYdP = (R * m_n).transpose();

        float residual = -(R * m_n).dot(P) - d;

#if USE_Z_AXIS
        m_PlaneH(i, 0) = dYdtheta.x();
        m_PlaneH(i, 1) = dYdtheta.y();
        m_PlaneH(i, 2) = 1.0f;
#else
        m_PlaneH(i, 0) = dYdtheta.x();
        m_PlaneH(i, 1) = dYdtheta.z();
        m_PlaneH(i, 2) = 1.0f;
#endif
        m_PlaneH.block(i, 3 + i * 6 + 3, 1, 3) = dYdP.transpose();
        m_PlaneH(i, residualIdx) = residual;

        i++;
        totalResidual += residual * residual;
    }

    printf("Residual :%f\n", totalResidual);

    m_PlaneH *= invStd;

    return nCams;
}
#endif

int SquareRootEKFSolver::stackInformationFactorMatrix() {
    int nTotalObs = 0;
    int nCamStartIdx = IMU_STATE_DIM;
    int nVisualObs = 0;
    int rffIdx = 0;
    int nCams = m_vCamStates.size();

#if USE_PLANE_PRIOR
    nCamStartIdx += PLANE_DIM;
    nTotalObs += _addPlaneContraint();
#endif

#if USE_KEYFRAME

    nVisualObs += _addNewSlamPointConstraint();
    nCamStartIdx += m_vSlamPoint.size() * 3;

#endif

#if USE_POSITION_DETECT_ROTATION

    bool addPositionContraint = false;
    if (!nVisualObs && _detectPureRotation() && !*m_bStatic) {
        addPositionContraint = true;
    }
#endif
    int nOldStates = CURRENT_DIM;
    int nCamStates = m_vCamStates.size() * CAM_STATE_DIM;

    using namespace std;
    for (auto& track : m_vMsckfPoints) {
        nVisualObs += track->H.rows();
    }

    nTotalObs += nVisualObs;

#if !USE_STATIC_STACK
    m_StackedMatrix.setZero(nTotalObs, nOldStates + 1);
#else
#if REMOVE_RESIDUAL_STACK
    m_StackedMatrix.topLeftCorner(nTotalObs, nOldStates).setZero();
    m_obsResidual.segment(0, nTotalObs).setZero();
#else
    m_StackedMatrix.topLeftCorner(nTotalObs, nOldStates + 1).setZero();
#endif
#endif

#if USE_PLANE_PRIOR

    m_StackedMatrix.topLeftCorner(nCams, 3) = m_PlaneH.topLeftCorner(nCams, 3);
    m_StackedMatrix.block(0, nCamStartIdx, nCams, CAM_STATE_DIM * nCams) =
        m_PlaneH.middleCols(3, CAM_STATE_DIM * nCams);
#if REMOVE_RESIDUAL_STACK
    m_obsResidual.segment(0, nCams) = m_PlaneH.rightCols<1>();
#else
    m_StackedMatrix.block(0, nOldStates, nCams, 1) = m_PlaneH.rightCols<1>();
#endif

    rffIdx = nCams;
#endif

    if (!nVisualObs) {
#if USE_STATIC_STACK
#if USE_PLANE_PRIOR
        m_nStackRows = nTotalObs;
#else
        m_nStackRows = 0;
#endif
#endif
        return 0;
    }

    float invSigma = 1 / sqrt(Config::ImageNoise2);
    MatrixXf H_j;

    for (auto& track : m_vMsckfPoints) {
        int nObs = track->H.rows();
        track->H *= invSigma;

        m_StackedMatrix.block(rffIdx, nCamStartIdx, nObs, nCamStates) =
            track->H.leftCols(nCamStates);
        //.triangularView<Eigen::Upper>();

#if REMOVE_RESIDUAL_STACK
        m_obsResidual.segment(rffIdx, nObs) = track->H.rightCols<1>();
#else
        m_StackedMatrix.block(rffIdx, nOldStates, nObs, 1) =
            track->H.rightCols<1>();
#endif
        rffIdx += nObs;
    }

    int slamPointIdx = IMU_STATE_DIM;

#if USE_PLANE_PRIOR
    slamPointIdx += PLANE_DIM;
#endif

#if USE_KEYFRAME

    for (auto& track : m_vSlamPoint) {
        if (track->m_bToMargin) continue;
        int nObs = track->H.rows();
        track->H *= invSigma;
        int slamId = track->m_idx;

        m_StackedMatrix.block(rffIdx, nCamStartIdx, nObs, nCamStates) =
            track->H.middleCols(3, nCamStates);
        m_StackedMatrix.block(rffIdx, slamPointIdx + slamId * 3, nObs, 3) =
            track->H.leftCols<3>();
#if REMOVE_RESIDUAL_STACK
        m_obsResidual.segment(rffIdx, nObs) = track->H.rightCols<1>();
#else
        m_StackedMatrix.block(rffIdx, nOldStates, nObs, 1) =
            track->H.rightCols<1>();
#endif
        rffIdx += nObs;
    }

#endif

#if USE_POSITION_DETECT_ROTATION

    if (addPositionContraint) {
        nTotalObs += _addPositionContraint(nTotalObs);
    }

#endif

#if USE_STATIC_STACK
    m_nStackRows = nTotalObs;
#endif
    return nTotalObs;
}

void SquareRootEKFSolver::solveAndUpdateStates() {
#if USE_STATIC_STACK
    int haveNewInformation = m_nStackRows;
#else
    int haveNewInformation = m_StackedMatrix.rows();
#endif
    if (haveNewInformation) {
#if USE_GIVENS_UPDATE
        TickTock::start("Givens");
        _updateByGivensRotations(haveNewInformation, CURRENT_DIM + 1);
        TickTock::stop("Givens");
#else

        Eigen::MatrixXf stackedMatrix;
        int nRows = CURRENT_DIM + m_StackedMatrix.rows();
        stackedMatrix.setZero(nRows, m_StackedMatrix.cols());
        stackedMatrix.topLeftCorner(CURRENT_DIM, CURRENT_DIM) =
            m_infoFactorMatrix.topLeftCorner(CURRENT_DIM, CURRENT_DIM);
        stackedMatrix.bottomRows(m_StackedMatrix.rows()) = m_StackedMatrix;

        Eigen::HouseholderQR<MatrixXf> qr(stackedMatrix);
        m_infoFactorMatrix.topLeftCorner(CURRENT_DIM, CURRENT_DIM) =
            qr.matrixQR()
                .topLeftCorner(CURRENT_DIM, CURRENT_DIM)
                .triangularView<Eigen::Upper>();

        m_residual = qr.matrixQR().topRightCorner(CURRENT_DIM, 1);
#endif

        TickTock::start("Inverse");

        VectorXf dx = m_infoFactorMatrix.topLeftCorner(CURRENT_DIM, CURRENT_DIM)
                          .triangularView<Eigen::Upper>()
                          .solve(m_residual.segment(0, CURRENT_DIM));
        TickTock::stop("Inverse");

        int iDim = 0;

#if USE_PLANE_PRIOR
        printf("Plane coeff:%f %f %f -> ", m_planeCoeff->x(), m_planeCoeff->y(),
               m_planeCoeff->z());
        *m_planeCoeff += dx.segment<3>(0);
        printf("%f %f %f\n", m_planeCoeff->x(), m_planeCoeff->y(),
               m_planeCoeff->z());
        iDim += PLANE_DIM;
#endif

#if !DISABLE_ACC_BIAS
        auto& imuBuffer = ImuBuffer::getInstance();
        imuBuffer.updateBias(dx.segment<3>(iDim), dx.segment<3>(iDim + 6));
        *m_pVel += dx.segment<3>(iDim + 3);
#else
        auto& imuBuffer = ImuBuffer::getInstance();
        imuBuffer.updateBias(dx.segment<3>(iDim), Vector3f(0, 0, 0));
        *m_pVel += dx.segment<3>(iDim + 3);
#endif
        iDim += IMU_STATE_DIM;

#if USE_KEYFRAME
        for (auto& pointState : m_vSlamPoint) {
            pointState->m_Pw += dx.segment<3>(iDim);
            iDim += 3;
        }
#endif

        for (auto& camState : m_vCamStates) {
            camState->m_Rwi =
                camState->m_Rwi *
                Sophus::SO3Group<float>::exp(dx.segment<3>(iDim)).matrix();
            iDim += 3;
            camState->m_Pwi += dx.segment<3>(iDim);
            iDim += 3;
        }
    }

#if USE_VELOCITY_DETECT_ROTATION
    m_pNewState->m_vel = *m_pVel;
#endif

    m_vMsckfPoints.clear();
}

#ifdef PLATFORM_ARM
void SquareRootEKFSolver::_updateByGivensRotationsNeon() {}

#endif

void SquareRootEKFSolver::_updateByGivensRotations(int row, int col) {
    for (int j = 0; j < col - 1; ++j) {
        for (int i = row - 1; i >= 0; --i) {
            if (i == 0) {
                float* pI = m_StackedMatrix.row(i).data();
                float alpha = m_infoFactorMatrix(j, j);
                float beta = pI[j];
                float c, s;
                if (fabs(beta) < FLT_EPSILON) {
                    continue;
                } else if (fabs(beta) > fabs(alpha)) {
                    s = 1 / sqrt(1 + pow(alpha / beta, 2));
                    c = -alpha / beta * s;
                } else {
                    c = 1 / sqrt(1 + pow(beta / alpha, 2));
                    s = -beta / alpha * c;
                }
                for (int k = j; k < col; ++k) {
                    if (k == col - 1) {
                        float x = m_residual(j);
#if REMOVE_RESIDUAL_STACK
                        float y = m_obsResidual(i);
                        m_obsResidual(i) = s * x + c * y;
#else
                        float y = pI[k];
                        pI[k] = s * x + c * y;
#endif
                        m_residual(j) = c * x + -s * y;

                    } else {
                        float x = m_infoFactorMatrix(j, k);
                        float y = pI[k];
                        m_infoFactorMatrix(j, k) = c * x + -s * y;
                        pI[k] = s * x + c * y;
                    }
                }
            } else {
                float* pJ = m_StackedMatrix.row(i - 1).data();
                float* pI = m_StackedMatrix.row(i).data();
                float alpha = pJ[j];
                float beta = pI[j];
                float c, s;
                if (fabs(beta) < FLT_EPSILON) {
                    continue;
                } else if (fabs(beta) > fabs(alpha)) {
                    s = 1 / sqrt(1 + pow(alpha / beta, 2));
                    c = -alpha / beta * s;
                } else {
                    c = 1 / sqrt(1 + pow(beta / alpha, 2));
                    s = -beta / alpha * c;
                }
                int k;
                for (k = j; k < col - 1; ++k) {
                    float x = pJ[k];
                    float y = pI[k];
                    pJ[k] = c * x + -s * y;
                    pI[k] = s * x + c * y;
                }
#if REMOVE_RESIDUAL_STACK
                float x = m_obsResidual[i - 1];
                float y = m_obsResidual[i];
                m_obsResidual[i - 1] = c * x + -s * y;
                m_obsResidual[i] = s * x + c * y;
#else
                float x = pJ[k];
                float y = pI[k];
                pJ[k] = c * x + -s * y;
                pI[k] = s * x + c * y;
#endif
            }
        }
    }
}

bool SquareRootEKFSolver::_detectPureRotation() {
#if USE_POSITION_DETECT_ROTATION

    static std::deque<Eigen::Vector3f> positionDeque;

    positionDeque.push_back(m_pNewState->m_Pwi);

    float max_position = 0;
    float position_thresh = 0.05 * 0.05;

    if (positionDeque.size() > 10) {
        positionDeque.pop_front();

        for (int i = 0; i < 10; i++) {
            for (int j = i + 1; j < 10; ++j) {
                float dP = (positionDeque[i] - positionDeque[j]).squaredNorm();
                if (dP > max_position) max_position = dP;
            }
        }

    } else
        return false;

    return max_position < position_thresh;

#endif

    return false;
}

int SquareRootEKFSolver::_addPositionContraint(int nRows) {
    static float invPosSigma = 1.0 / 0.05;

#if USE_STATIC_STACK
    m_StackedMatrix.middleRows(nRows, 3).setZero();
#else
    m_StackedMatrix.conservativeResize(nRows + 3, Eigen::NoChange);
    m_StackedMatrix.bottomRows<3>().setZero();
#endif

    Vector3f dP = m_pNewState->m_Pwi - m_pLastState->m_Pwi;

    m_StackedMatrix.block<3, 3>(nRows + 3,
                                CURRENT_DIM - 2 * CAM_STATE_DIM + 3) =
        -Matrix3f::Identity() * invPosSigma;
    m_StackedMatrix.block<3, 3>(nRows + 3, CURRENT_DIM - CAM_STATE_DIM + 3) =
        Matrix3f::Identity() * invPosSigma;
#if REMOVE_RESIDUAL_STACK
    m_obsResidual.segment(nRows + 3, 3) = -dP * invPosSigma;
#else
    m_StackedMatrix.block<3, 1>(nRows + 3, CURRENT_DIM) = -dP * invPosSigma;
#endif
    return 3;
}

#if USE_NEW_MOVED_PIXEL
void SquareRootEKFSolver::_computeDeltaR() {
    for (auto& camState : m_vCamStates) {
        camState->m_dR.resize(m_vCamStates.size());
    }

    Matrix3f dR;
    for (int i = 0, n = m_vCamStates.size(); i < n; ++i) {
        auto camState = m_vCamStates[i];
        camState->m_dR[i].setIdentity();
        for (int j = i + 1; j < n; ++j) {
            auto camState2 = m_vCamStates[j];
            dR = camState2->m_Rwi.transpose() * camState->m_Rwi;
            camState->m_dR[j] = dR;
            camState2->m_dR[i] = dR.transpose();
        }
    }
}
#endif
void SquareRootEKFSolver::_marginByGivensRotation() {
    int nRows = CURRENT_DIM;
    for (int j = 0; j < CURRENT_DIM; ++j) {
        for (int i = nRows - 1; i > j; --i) {
            float* pJ = m_infoFactorMatrixToMarginal.row(i - 1).data();
            float* pI = m_infoFactorMatrixToMarginal.row(i).data();
            float alpha = pJ[j];
            float beta = pI[j];
            float c, s;
            if (fabs(beta) < FLT_EPSILON) {
                continue;
            } else if (fabs(alpha) < FLT_EPSILON) {
                for (int k = j; k < CURRENT_DIM; ++k) {
                    std::swap(pJ[k], pI[k]);
                }
                continue;
                ;
            } else if (fabs(beta) > fabs(alpha)) {
                s = 1 / sqrt(1 + pow(alpha / beta, 2));
                c = -alpha / beta * s;
            } else {
                c = 1 / sqrt(1 + pow(beta / alpha, 2));
                s = -beta / alpha * c;
            }
            for (int k = j; k < CURRENT_DIM; ++k) {
                float x = pJ[k];
                float y = pI[k];
                pJ[k] = c * x - s * y;
                pI[k] = s * x + c * y;
            }
        }
    }
}

void SquareRootEKFSolver::marginalizeGivens() {
    int iDim = 0;
    std::vector<int> v_MarginDIM, v_RemainDIM;
    std::vector<CamState*> v_CamStateNew;
    std::vector<PointState*> v_PointStateNew;

#if USE_PLANE_PRIOR

    v_RemainDIM.push_back(0);
    v_RemainDIM.push_back(1);
    v_RemainDIM.push_back(2);

    for (int i = 0; i < IMU_STATE_DIM; ++i) {
        v_MarginDIM.push_back(i + PLANE_DIM);
        v_RemainDIM.push_back(CURRENT_DIM - IMU_STATE_DIM + i);
    }
    iDim = IMU_STATE_DIM + PLANE_DIM;

#else
    for (int i = 0; i < IMU_STATE_DIM; ++i) {
        v_MarginDIM.push_back(i);
        v_RemainDIM.push_back(CURRENT_DIM - IMU_STATE_DIM + i);
    }
    iDim = IMU_STATE_DIM;
#endif
#if USE_KEYFRAME

    for (int i = 0, n = m_vSlamPoint.size(); i < n; ++i) {
        if (m_vSlamPoint[i]->m_bToMargin || m_vSlamPoint[i]->m_bNextMargin)
            m_vSlamPoint[i]->host->m_bDead = true;
        if (m_vSlamPoint[i]->host->m_bDead) {
            for (int j = 0; j < 3; ++j) {
                v_MarginDIM.push_back(iDim++);
            }
        } else {
            for (int j = 0; j < 3; ++j) {
                v_RemainDIM.push_back(iDim++);
            }
            v_PointStateNew.push_back(m_vSlamPoint[i]);
        }
    }

    m_vSlamPoint = v_PointStateNew;
    for (int i = 0, n = m_vSlamPoint.size(); i < n; ++i) {
        m_vSlamPoint[i]->m_idx = i;
    }
#endif

    for (int i = 0, n = m_vCamStates.size(); i < n; ++i) {
        auto state = m_vCamStates[i];
        if (state->m_bToMargin)
            for (int i = 0; i < CAM_STATE_DIM; i++)
                v_MarginDIM.push_back(iDim++);
        else {
            for (int i = 0; i < CAM_STATE_DIM; i++)
                v_RemainDIM.push_back(iDim++);
            v_CamStateNew.push_back(state);
        }
    }

    m_vCamStates = v_CamStateNew;
    m_vCamStates.push_back(m_pNewState);
    for (int i = 0, n = m_vCamStates.size(); i < n; ++i) {
        m_vCamStates[i]->m_idx = i;
    }

    for (int i = 0; i < CAM_STATE_DIM; i++) {
        v_RemainDIM.push_back(iDim++);
    }

    // rearrange new information factor matrix

    int index = 0;
    for (auto idx : v_MarginDIM) {
        m_infoFactorMatrixToMarginal.col(index++).segment(0, CURRENT_DIM) =
            m_infoFactorMatrix.col(idx).segment(0, CURRENT_DIM);
    }
    for (auto idx : v_RemainDIM) {
        m_infoFactorMatrixToMarginal.col(index++).segment(0, CURRENT_DIM) =
            m_infoFactorMatrix.col(idx).segment(0, CURRENT_DIM);
    }

    // use qr to marginalize states

    _marginByGivensRotation();

    int OLD_DIM = CURRENT_DIM;
    CURRENT_DIM = v_RemainDIM.size();

#if USE_KEYFRAME

    m_infoFactorMatrixAfterMarigin.topLeftCorner(CURRENT_DIM, CURRENT_DIM) =
        m_infoFactorMatrixToMarginal.block(OLD_DIM - CURRENT_DIM,
                                           OLD_DIM - CURRENT_DIM, CURRENT_DIM,
                                           CURRENT_DIM);

#else

    m_infoFactorMatrix.topLeftCorner(CURRENT_DIM, CURRENT_DIM) =
        m_infoFactorMatrixToMarginal.block(OLD_DIM - CURRENT_DIM,
                                           OLD_DIM - CURRENT_DIM, CURRENT_DIM,
                                           CURRENT_DIM);

#endif

#if USE_NEW_MOVED_PIXEL

    _computeDeltaR();
#endif
}

void SquareRootEKFSolver::marginalizeStatic() {
    int iDim = 0;
    std::vector<int> v_MarginDIM, v_RemainDIM;
    std::vector<CamState*> v_CamStateNew;

    for (int i = 0; i < IMU_STATE_DIM; ++i) {
        v_MarginDIM.push_back(i);
        v_RemainDIM.push_back(CURRENT_DIM - IMU_STATE_DIM + i);
    }
    iDim = IMU_STATE_DIM;
    for (int i = 0, n = m_vCamStates.size(); i < n; ++i) {
        auto state = m_vCamStates[i];
        if (state->m_bToMargin)
            for (int i = 0; i < CAM_STATE_DIM; i++)
                v_MarginDIM.push_back(iDim++);
        else {
            for (int i = 0; i < CAM_STATE_DIM; i++)
                v_RemainDIM.push_back(iDim++);
            v_CamStateNew.push_back(state);
        }
    }
    m_vCamStates = v_CamStateNew;
    m_vCamStates.push_back(m_pNewState);
    for (int i = 0, n = m_vCamStates.size(); i < n; ++i) {
        m_vCamStates[i]->m_idx = i;
    }

    for (int i = 0; i < CAM_STATE_DIM; i++) {
        v_RemainDIM.push_back(iDim++);
    }

    // rearrange new information factor matrix

    int index = 0;
    for (auto idx : v_MarginDIM) {
        m_infoFactorMatrixToMarginal.col(index++) = m_infoFactorMatrix.col(idx);
    }
    for (auto idx : v_RemainDIM) {
        m_infoFactorMatrixToMarginal.col(index++) = m_infoFactorMatrix.col(idx);
    }

    // use qr to marginalize states

    Eigen::HouseholderQR<MatrixXf> qr(
        m_infoFactorMatrixToMarginal.topLeftCorner(CURRENT_DIM, CURRENT_DIM));
    CURRENT_DIM = v_RemainDIM.size();

    m_infoFactorMatrix.topLeftCorner(CURRENT_DIM, CURRENT_DIM) =
        qr.matrixQR()
            .bottomRightCorner(CURRENT_DIM, CURRENT_DIM)
            .triangularView<Eigen::Upper>();

    // m_infoFactorInverseMatrix = MatrixXf::Identity(CURRENT_DIM, CURRENT_DIM);
    // m_infoFactorMatrix.topLeftCorner(CURRENT_DIM,
    // CURRENT_DIM).triangularView<Eigen::Upper>().solveInPlace(m_infoFactorInverseMatrix);

    m_residual.segment(0, CURRENT_DIM).setZero();
}

}  // namespace DeltaVins
