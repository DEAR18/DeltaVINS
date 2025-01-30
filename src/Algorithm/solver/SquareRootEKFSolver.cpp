#include "Algorithm/solver/SquareRootEKFSolver.h"

#include <sophus/so3.hpp>

#include "Algorithm/IMU/ImuPreintergration.h"
#include "Algorithm/vision/camModel/camModel.h"
#include "IO/dataBuffer/imuBuffer.h"
#include "precompile.h"
#include "utils/utils.h"
#include "utils/TickTock.h"
#include "utils/constantDefine.h"

namespace DeltaVins {
SquareRootEKFSolver::SquareRootEKFSolver() {
    // info_factor_matrix_.resize(MAX_MATRIX_SIZE,MAX_MATRIX_SIZE);
    // info_factor_matrix_to_marginal_.resize(MAX_MATRIX_SIZE,MAX_MATRIX_SIZE);
    // residual_.resize(MAX_MATRIX_SIZE,1);
    //
    //
}

void SquareRootEKFSolver::Init(CamState* pCamState, Vector3f* vel
#if USE_PLANE_PRIOR
                               ,
                               Vector3f* planeCoeff, Vector3f n
#endif
                               ,
                               bool* static_) {

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

    vel_ = vel;
    static_ = static_;
#if USE_PLANE_PRIOR

    m_planeCoeff = planeCoeff;
    m_n = n;
#endif
    info_factor_matrix_.topLeftCorner(CURRENT_DIM, CURRENT_DIM) =
        p.cwiseSqrt().cwiseInverse().asDiagonal();

    vel_->setZero();
    AddCamState(pCamState);
    cam_states_.push_back(pCamState);
}

void SquareRootEKFSolver::AddCamState(CamState* state) {
    last_state_ = new_state_;
    new_state_ = state;
}
void SquareRootEKFSolver::PropagateStatic(const ImuPreintergration* imu_term) {
#if USE_Z_AXIS
    static Vector3f gravity(0, 0, GRAVITY);
#else
    static Vector3f gravity(0, GRAVITY, 0);
#endif
    Matrix3f R0 = last_state_->Rwi;

    float dt = imu_term->dT * 1e-9;

    // propagate states
    new_state_->Pwi =
        last_state_->Pwi +
        (R0 * imu_term->dP + *vel_ * dt + gravity * (0.5f * dt * dt));
    new_state_->Pw_FEJ = new_state_->Pwi;
    *vel_ += R0 * imu_term->dV + gravity * dt;
    new_state_->Rwi = R0 * imu_term->dR;

    // Make state transition matrix F
    static Eigen::Matrix<float, NEW_STATE_DIM, NEW_STATE_DIM>
        state_transition_matrix;
    state_transition_matrix.setIdentity();
    state_transition_matrix.block<3, 3>(0, 0) = imu_term->dR.transpose();
    state_transition_matrix.block<3, 3>(0, 6) = imu_term->dRdg;
    state_transition_matrix.block<3, 3>(3, 0) = -R0 * crossMat(imu_term->dP);
    state_transition_matrix.block<3, 3>(3, 6) = R0 * imu_term->dPdg;
    state_transition_matrix.block<3, 3>(3, 9) = Matrix3f::Identity() * dt;
    state_transition_matrix.block<3, 3>(9, 0) = -R0 * crossMat(imu_term->dV);
    state_transition_matrix.block<3, 3>(9, 6) = R0 * imu_term->dVdg;
#if !DISABLE_ACC_BIAS
    state_transition_matrix.block<3, 3>(9, 12) = R0 * imu_term->dVda;
    state_transition_matrix.block<3, 3>(3, 12) = R0 * imu_term->dPda;
#endif

#if USE_SQ_NOISE

    // make noise transition matrix
    static Eigen::Matrix<float, 9, 9> noise_transition_matrix;
    noise_transition_matrix.setZero();
    noise_transition_matrix.block<3, 3>(0, 0).setIdentity();
    noise_transition_matrix.block<3, 3>(3, 6) = R0;
    noise_transition_matrix.block<3, 3>(6, 3) = R0;

    Eigen::LLT<Matrix9f> chol(imu_term->Cov);
    Matrix9f covFactor;
    covFactor.setIdentity();
    chol.matrixU().solveInPlace(covFactor);
    covFactor = noise_transition_matrix * covFactor;
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
    static Eigen::Matrix<float, NEW_STATE_DIM, 9> noise_transition_matrix;
    noise_transition_matrix.setZero();
    noise_transition_matrix.block<3, 3>(0, 0).setIdentity();
    noise_transition_matrix.block<3, 3>(3, 6) = R0;
    noise_transition_matrix.block<3, 3>(9, 3) = R0;

    // Make Noise Covariance Matrix Q
    static Eigen::Matrix<float, NEW_STATE_DIM, NEW_STATE_DIM> noise_cov;
    noise_cov = noise_transition_matrix * imu_term->Cov *
               noise_transition_matrix.transpose();
    noise_cov.block<3, 3>(6, 6) =
        Matrix3f::Identity() * (Config::GyroBiasNoise2 * dt * dt);
#if !DISABLE_ACC_BIAS
    noise_cov.block<3, 3>(12, 12) =
        Matrix3f::Identity() * (Config::AccBiasNoise2 * dt * dt);
#endif

    static Eigen::Matrix<float, NEW_STATE_DIM, NEW_STATE_DIM> NoiseFactor;
    NoiseFactor.setIdentity();
    Eigen::LLT<MatrixXf> chol(noise_cov);
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
    info_factor_matrix_.block(0, OLD_DIM, OLD_DIM, NEW_STATE_DIM).setZero();
    info_factor_matrix_.block(OLD_DIM, 0, NEW_STATE_DIM, OLD_DIM).setZero();
    MatrixXf R = NoiseFactor * state_transition_matrix;
    info_factor_matrix_.block<NEW_STATE_DIM, IMU_STATE_DIM>(OLD_DIM, IMUIdx) =
        R.rightCols<IMU_STATE_DIM>();
    info_factor_matrix_.block<NEW_STATE_DIM, CAM_STATE_DIM>(
        OLD_DIM, OLD_DIM - CAM_STATE_DIM) = R.leftCols<CAM_STATE_DIM>();
    // info_factor_matrix_.block<NEW_STATE_DIM, NEW_STATE_DIM>(N, N -
    // NEW_STATE_DIM) = NoiseFactor * state_transition_matrix;
    info_factor_matrix_.block<NEW_STATE_DIM, NEW_STATE_DIM>(OLD_DIM, OLD_DIM) =
        -NoiseFactor;

    // make residual vector
    residual_.segment(0, CURRENT_DIM).setZero();

#if USE_VELOCITY_DETECT_ROTATION
    new_state_->vel = *vel_;
#endif
}
#if 0
	void SquareRootEKFSolver::Propagate(const ImuPreintergration* imu_term)
    {

        static Vector3f gravity(0, GRAVITY, 0);
        Matrix3f R0 = last_state_->Rwi;

        float dt = imu_term->dT * 1e-9;

    	// propagate states
        new_state_->Pwi = last_state_->Pwi + (R0 * imu_term->dP + *vel_ * dt + gravity * (0.5f * dt * dt));
        new_state_->Pw_FEJ = new_state_->Pwi;
        *vel_ += R0 * imu_term->dV + gravity * dt;
        new_state_->Rwi = R0 * imu_term->dR;

        //Make state transition matrix F
        Eigen::Matrix<float,NEW_STATE_DIM,NEW_STATE_DIM> state_transition_matrix;
        state_transition_matrix.setIdentity();
        state_transition_matrix.block<3, 3>(0, 0) = imu_term->dR.transpose();
        state_transition_matrix.block<3, 3>(0, 6) = imu_term->dRdg;
        state_transition_matrix.block<3, 3>(3, 0) = -R0 * crossMat(imu_term->dP);
        state_transition_matrix.block<3, 3>(3, 6) = R0 * imu_term->dPdg;
        state_transition_matrix.block<3, 3>(3, 9) = Matrix3f::Identity() * dt;
        state_transition_matrix.block<3, 3>(9, 0) = -R0 * crossMat(imu_term->dV);
        state_transition_matrix.block<3, 3>(9, 6) = R0 * imu_term->dVdg;
#if !DISABLE_ACC_BIAS
        state_transition_matrix.block<3, 3>(9, 12) = R0 * imu_term->dVda;
        state_transition_matrix.block<3, 3>(3, 12) = R0 * imu_term->dPda;
#endif

        //make noise transition matrix
        Eigen::Matrix<float,NEW_STATE_DIM,9> noise_transition_matrix;
        noise_transition_matrix.setZero();
        noise_transition_matrix.block<3, 3>(0, 0).setIdentity();
        noise_transition_matrix.block<3, 3>(3, 6) = R0;
        noise_transition_matrix.block<3, 3>(9, 3) = R0;

        //Make Noise Covariance Matrix Q
        Eigen::Matrix<float,NEW_STATE_DIM,NEW_STATE_DIM> noise_cov = noise_transition_matrix * imu_term->Cov * noise_transition_matrix.transpose();
        noise_cov.block<3, 3>(6, 6) = Matrix3f::Identity() * (Config::GyroBiasNoise2 * dt * dt);
#if !DISABLE_ACC_BIAS
        noise_cov.block<3, 3>(12, 12) = Matrix3f::Identity() * (Config::AccBiasNoise2 * dt * dt);
#endif

        Eigen::Matrix<float,NEW_STATE_DIM,NEW_STATE_DIM> NoiseFactor;
        NoiseFactor.setIdentity();
        Eigen::LLT<MatrixXf> chol(noise_cov);
        chol.matrixU().solveInPlace(NoiseFactor);

        //Make information factor matrix
        int N = info_factor_matrix_.rows();
        info_factor_matrix_.conservativeResize(N + NEW_STATE_DIM, N + NEW_STATE_DIM);

    	MatrixXf A = NoiseFactor * state_transition_matrix;

        info_factor_matrix_.topRightCorner(N, NEW_STATE_DIM).setZero();
        info_factor_matrix_.bottomLeftCorner(NEW_STATE_DIM, N).setZero();
        info_factor_matrix_.block<NEW_STATE_DIM, NEW_STATE_DIM>(N, N - NEW_STATE_DIM) = A;
        info_factor_matrix_.bottomRightCorner<NEW_STATE_DIM, NEW_STATE_DIM>() = -NoiseFactor;



        // make residual vector
        residual_ = VectorXf::Zero(info_factor_matrix_.rows());

    }

    void SquareRootEKFSolver::PropagateNew(const ImuPreintergration* imu_term)
    {
        static Vector3f gravity(0, GRAVITY, 0);
        Matrix3f R0 = last_state_->Rwi;

        float dt = imu_term->dT * 1e-9;

    	// propagate states
        new_state_->Pwi = last_state_->Pwi + (R0 * imu_term->dP + *vel_ * dt + gravity * (0.5f * dt * dt));
        new_state_->Pw_FEJ = new_state_->Pwi;
        *vel_ += R0 * imu_term->dV + gravity * dt;
        new_state_->Rwi = R0 * imu_term->dR;

        //Make state transition matrix F
        Eigen::Matrix<float, NEW_STATE_DIM, NEW_STATE_DIM> state_transition_matrix;
        state_transition_matrix.setIdentity();
        state_transition_matrix.block<3, 3>(0, 0) = imu_term->dR.transpose();
        state_transition_matrix.block<3, 3>(0, 6) = imu_term->dRdg;
        state_transition_matrix.block<3, 3>(3, 0) = -R0 * crossMat(imu_term->dP);
        state_transition_matrix.block<3, 3>(3, 6) = R0 * imu_term->dPdg;
        state_transition_matrix.block<3, 3>(3, 9) = Matrix3f::Identity() * dt;
        state_transition_matrix.block<3, 3>(3, 12) = R0 * imu_term->dPda;
        state_transition_matrix.block<3, 3>(9, 0) = -R0 * crossMat(imu_term->dV);
        state_transition_matrix.block<3, 3>(9, 6) = R0 * imu_term->dVdg;
        state_transition_matrix.block<3, 3>(9, 12) = R0 * imu_term->dVda;

        //make noise transition matrix
        Eigen::Matrix<float, NEW_STATE_DIM, 9> noise_transition_matrix;
        noise_transition_matrix.setZero();
        noise_transition_matrix.block<3, 3>(0, 0).setIdentity();
        noise_transition_matrix.block<3, 3>(3, 6) = R0;
        noise_transition_matrix.block<3, 3>(9, 3) = R0;

        //Make Noise Covariance Matrix Q
        Eigen::Matrix<float, NEW_STATE_DIM, NEW_STATE_DIM> noise_cov = noise_transition_matrix * imu_term->Cov * noise_transition_matrix.transpose();
        noise_cov.block<3, 3>(6, 6) = Matrix3f::Identity() * (Config::GyroBiasNoise2 * dt * dt);
        noise_cov.block<3, 3>(12, 12) = Matrix3f::Identity() * (Config::AccBiasNoise2 * dt * dt);

        Eigen::Matrix<float, NEW_STATE_DIM, NEW_STATE_DIM> NoiseFactor;
        NoiseFactor.setIdentity();
        Eigen::LLT<MatrixXf> chol(noise_cov);
        chol.matrixU().solveInPlace(NoiseFactor);

        //Make information factor matrix
        int N = info_factor_matrix_.rows();
        info_factor_matrix_.conservativeResize(N + NEW_STATE_DIM, N + NEW_STATE_DIM);

        info_factor_matrix_.topRightCorner(N, NEW_STATE_DIM).setZero();
        info_factor_matrix_.bottomLeftCorner(NEW_STATE_DIM, N).setZero();
        MatrixXf R = NoiseFactor * state_transition_matrix;
        info_factor_matrix_.block<NEW_STATE_DIM,IMU_STATE_DIM>(N, 0) = R.rightCols<IMU_STATE_DIM>();
        info_factor_matrix_.block<NEW_STATE_DIM, CAM_STATE_DIM>(N, N-CAM_STATE_DIM) = R.leftCols<CAM_STATE_DIM>();
        //info_factor_matrix_.block<NEW_STATE_DIM, NEW_STATE_DIM>(N, N - NEW_STATE_DIM) = NoiseFactor * state_transition_matrix;
        info_factor_matrix_.bottomRightCorner<NEW_STATE_DIM, NEW_STATE_DIM>() = -NoiseFactor;

        // make residual vector
        residual_ = VectorXf::Zero(info_factor_matrix_.rows());


    }

    void SquareRootEKFSolver::marginalize()
    {
        int iDim = 0;
        std::vector<int> v_MarginDIM,v_RemainDIM;
        std::vector<CamState*> v_CamStateNew;
        for (int i=0,n = cam_states_.size();i<n;++i)
        {
            auto state = cam_states_[i];
            if (state->flag_to_marginalize)
                for (int i = 0; i < CAM_STATE_DIM; i++)
                    v_MarginDIM.push_back(iDim++);
            else
            {
                for (int i = 0; i < CAM_STATE_DIM; i++)
                    v_RemainDIM.push_back(iDim++);
                v_CamStateNew.push_back(state);
            }
        }
        cam_states_ = v_CamStateNew;
        cam_states_.push_back(new_state_);
        for (int i=0,n=cam_states_.size();i<n;++i)
        {
            cam_states_[i]->index_in_window = i;
        }

        for (int i = 0; i < IMU_STATE_DIM; i++)
        {
            v_MarginDIM.push_back(iDim++);
        }
        for (int i = 0; i < NEW_STATE_DIM; i++)
        {
            v_RemainDIM.push_back(iDim++);
        }

        int nCols = info_factor_matrix_.cols();

        //rearrange new information factor matrix
        MatrixXf NewInfoFactor(nCols, nCols);
        int index = 0;
        for (auto idx : v_MarginDIM)
        {
            NewInfoFactor.col(index++) = info_factor_matrix_.col(idx);
        }
        for (auto idx : v_RemainDIM)
        {
            NewInfoFactor.col(index++) = info_factor_matrix_.col(idx);
        }

        //use qr to marginalize states
        Eigen::HouseholderQR<MatrixXf> qr(NewInfoFactor);
        int nRemain = v_RemainDIM.size();

        info_factor_matrix_ = qr.matrixQR().bottomRightCorner(nRemain, nRemain).triangularView<Eigen::Upper>();

        m_infoFactorInverseMatrix = MatrixXf::Identity(nRemain, nRemain);
        info_factor_matrix_.triangularView<Eigen::Upper>().solveInPlace(m_infoFactorInverseMatrix);

        residual_.resize(info_factor_matrix_.rows(), 1);
        residual_.setZero();



    }

    void SquareRootEKFSolver::marginalizeNew()
    {
        int iDim = 0;
        std::vector<int> v_MarginDIM, v_RemainDIM;
        std::vector<CamState*> v_CamStateNew;

        int N = info_factor_matrix_.cols();
        for (int i=0;i<IMU_STATE_DIM;++i)
        {
            v_MarginDIM.push_back(i);
            v_RemainDIM.push_back(N - IMU_STATE_DIM + i);
        }
        iDim = IMU_STATE_DIM;
        for (int i = 0, n = cam_states_.size(); i < n; ++i)
        {
            auto state = cam_states_[i];
            if (state->flag_to_marginalize)
                for (int i = 0; i < CAM_STATE_DIM; i++)
                    v_MarginDIM.push_back(iDim++);
            else
            {
                for (int i = 0; i < CAM_STATE_DIM; i++)
                    v_RemainDIM.push_back(iDim++);
                v_CamStateNew.push_back(state);
            }
        }
        cam_states_ = v_CamStateNew;
        cam_states_.push_back(new_state_);
        for (int i = 0, n = cam_states_.size(); i < n; ++i)
        {
            cam_states_[i]->index_in_window = i;
        }

        for (int i = 0; i < CAM_STATE_DIM; i++)
        {
            v_RemainDIM.push_back(iDim++);
        }

        int nCols = info_factor_matrix_.cols();

        //rearrange new information factor matrix
        MatrixXf NewInfoFactor(nCols, nCols);
        int index = 0;
        for (auto idx : v_MarginDIM)
        {
            NewInfoFactor.col(index++) = info_factor_matrix_.col(idx);
        }
        for (auto idx : v_RemainDIM)
        {
            NewInfoFactor.col(index++) = info_factor_matrix_.col(idx);
        }





        //use qr to marginalize states
        Eigen::HouseholderQR<MatrixXf> qr(NewInfoFactor);
        int nRemain = v_RemainDIM.size();

        info_factor_matrix_ = qr.matrixQR().bottomRightCorner(nRemain, nRemain).triangularView<Eigen::Upper>();
    	

        m_infoFactorInverseMatrix = MatrixXf::Identity(nRemain, nRemain);
        info_factor_matrix_.triangularView<Eigen::Upper>().solveInPlace(m_infoFactorInverseMatrix);

        residual_.resize(info_factor_matrix_.rows(), 1);
        residual_.setZero();

    }

#endif

bool SquareRootEKFSolver::MahalanobisTest(PointState* state) {
    VectorXf z = state->H.rightCols<1>();
    int nExceptPoint = state->H.cols() - 1;
    int num_obs = z.rows();
#if USE_NAIVE_ML_DATAASSOCIATION
    float phi;
#if USE_KEYFRAME
    if (state->flag_slam_point) {
        Matrix2f S;
        Matrix2f R = MatrixXf::Identity(2, 2) * Config::ImageNoise2 * 2;
        MatrixXf E;
        E.resize(CURRENT_DIM, 9);
#if USE_PLANE_PRIOR
        int iLeft = IMU_STATE_DIM + PLANE_DIM;
#else
        int iLeft = IMU_STATE_DIM;
#endif
        E.leftCols<3>() = info_factor_matrix_after_mariginal_.block(
            0, state->index_in_window * 3 + iLeft, CURRENT_DIM, 3);
        E.rightCols<6>() = info_factor_matrix_after_mariginal_.block(
            0,
            state->host->visual_obs.back().link_frame->state->index_in_window *
                    CAM_STATE_DIM +
                iLeft + 3 * slam_point_.size(),
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
    MatrixXf S = MatrixXf::Zero(num_obs, num_obs);
    MatrixXf R = MatrixXf::Identity(num_obs, num_obs) * Config::ImageNoise2 * 2;

    MatrixXf B =
        state->H.leftCols(nExceptPoint) *
        m_infoFactorInverseMatrix.topLeftCorner(nExceptPoint, nExceptPoint)
            .triangularView<Eigen::Upper>();
    nn S.noalias() += R;

    float phi = z.transpose() * S.llt().solve(z);
#endif

    assert(num_obs < 80);
#if OUTPUT_DEBUG_INFO
    if (phi < chi2LUT[num_obs])
        printf("#### MahalanobisTest Success\n");
    else
        printf("#### MahalanobisTest Fail\n");

#endif
    return phi < chi2LUT[num_obs];
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

int SquareRootEKFSolver::ComputeJacobians(TrackedFeature* track) {
    // observation number
    int index = 0;

    static CamModel* cam_model = CamModel::getCamModel();
    static Vector3f Tci = cam_model->getTci();
    int num_cams = cam_states_.size();

    const int CAM_STATE_IDX = 3;

    const int RESIDUAL_IDX = 3 + CAM_STATE_DIM * num_cams;

    int num_obs = track->visual_obs.size();

#if USE_KEYFRAME
    if (track->point_state_->flag_slam_point) {
        num_obs = 1;
    }
#endif
    float huberThresh = 500.f;
    float cutOffThresh = 10.f;

    // Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> H;
    auto& H = track->point_state_->H;
    H.setZero(num_obs * 2, 3 + (CAM_STATE_DIM * num_cams) + 1);
    auto calcObsJac = [&](VisualObservation* ob) {
        int cam_id = ob->link_frame->state->index_in_window;

        Matrix3f Riw = ob->link_frame->state->Rwi.transpose();
        Vector3f Pi =
            Riw * (track->point_state_->Pw - ob->link_frame->state->Pwi);
        Vector3f Pi_FEJ = Riw * (track->point_state_->Pw_FEJ -
                                 ob->link_frame->state->Pw_FEJ);

        Vector2f px = cam_model->imuToImage(Pi);
        Vector2f r = ob->px - px;
        float reprojErr = r.norm();

        if (reprojErr > cutOffThresh) {
            track->Reproject();
            // track->DrawObservationsAndReprojection(1);
            printf("%f %f ->%f %f\n", ob->px.x(), ob->px.y(), px.x(),
                   px.y());
            return false;
        }

        H.block<2, 1>(2 * index, RESIDUAL_IDX) = r;

        Matrix23f J23;
        cam_model->camToImage(Pi_FEJ + Tci, J23);

        H.block<2, 3>(2 * index, CAM_STATE_IDX + CAM_STATE_DIM * cam_id) =
            J23 * crossMat(Pi_FEJ);
#if USE_POSITION_DETECT_ROTATION
        if (!track->m_bInaccurateDepth)
#endif
            H.block<2, 3>(2 * index, CAM_STATE_IDX + CAM_STATE_DIM * cam_id + 3) =
                -J23 * Riw;
        H.block<2, 3>(2 * index, 0) = J23 * Riw;

        return true;
    };

#if USE_KEYFRAME
    if (track->point_state_->flag_slam_point) {
        if (calcObsJac(&track->visual_obs.back())) {
            index++;
        }
        if (index != num_obs) {
            return 0;
        }
    } else {
#endif

        for (auto& ob : track->visual_obs) {
            if (calcObsJac(&ob)) {
                index++;
            }
        }

#if USE_KEYFRAME
    }

#endif
    if (index != num_obs) {
        if (index < 2) return 0;
        num_obs = index;
        H.conservativeResize(2 * index, Eigen::NoChange);
    }

    return 2 * num_obs;
}

#if USE_KEYFRAME

int SquareRootEKFSolver::_AddNewSlamPointConstraint() {
    int num_obs = 0;

    for (auto point : slam_point_) {
        int obs = ComputeJacobians(point->host);

        if (obs && MahalanobisTest(point))
            num_obs += obs;
        else
            point->flag_to_marginalize = true;
    }

    if (new_slam_point_.empty()) {
        info_factor_matrix_.topLeftCorner(CURRENT_DIM, CURRENT_DIM) =
            info_factor_matrix_after_mariginal_.topLeftCorner(CURRENT_DIM,
                                                         CURRENT_DIM);
        return num_obs;
    }

    int nNewSlamPointStates = new_slam_point_.size() * 3;

    int nOldSlamPointStates = slam_point_.size() * 3;
    int num_cams = cam_states_.size();

#if USE_PLANE_PRIOR
    int nLeft = nOldSlamPointStates + IMU_STATE_DIM + PLANE_DIM;
#else
    int nLeft = nOldSlamPointStates + IMU_STATE_DIM;
#endif

    int nRight = num_cams * CAM_STATE_DIM;

    int nNewCam = nLeft + nNewSlamPointStates;

    info_factor_matrix_.topLeftCorner(nLeft, nLeft) =
        info_factor_matrix_after_mariginal_.topLeftCorner(nLeft, nLeft);
    info_factor_matrix_.block(0, nNewCam, nLeft, nRight) =
        info_factor_matrix_after_mariginal_.block(0, nLeft, nLeft, nRight);
    info_factor_matrix_.block(nNewCam, 0, nRight, nLeft) =
        info_factor_matrix_after_mariginal_.block(nLeft, 0, nRight, nLeft);
    info_factor_matrix_.block(nNewCam, nNewCam, nRight, nRight) =
        info_factor_matrix_after_mariginal_.block(nLeft, nLeft, nRight, nRight);

    info_factor_matrix_
        .block(0, nLeft, CURRENT_DIM + nNewSlamPointStates, nNewSlamPointStates)
        .setZero();
    info_factor_matrix_
        .block(nLeft, 0, nNewSlamPointStates, CURRENT_DIM + nNewSlamPointStates)
        .setZero();

    CURRENT_DIM += nNewSlamPointStates;

    for (auto state : new_slam_point_) {
        state->host->RemoveUselessObservationForSlamPoint();
    }

    for (auto& state : new_slam_point_) {
        num_obs += state->H.rows();
    }

    slam_point_.insert(slam_point_.end(), new_slam_point_.begin(),
                        new_slam_point_.end());

    for (int i = 0, n = slam_point_.size(); i < n; ++i) {
        slam_point_[i]->index_in_window = i;
    }

    new_slam_point_.clear();

    return num_obs;
}

int SquareRootEKFSolver::AddSlamPointConstraint() {
    int num_obs = 0;
    return num_obs;
}

#endif

void SquareRootEKFSolver::AddMsckfPoint(PointState* state) {
    // constexpr int MAX_DIM =
        // 3 + (CAM_STATE_DIM * MAX_WINDOW_SIZE + IMU_STATE_DIM + 1);
    static MatrixHfR H;
    // do null space trick to get pose constraint
    int col = state->H.cols();
    int row = state->H.rows();
    H.topLeftCorner(row, col) = state->H;

    rowMajorMatrixQRByGivensInMsckf(H, row, col);
    state->H = H.block(3, 3, row - 3, col - 3);

    msckf_points_.push_back(state);
}

#if USE_KEYFRAME
void SquareRootEKFSolver::AddSlamPoint(PointState* state) {
    new_slam_point_.push_back(state);
    state->flag_slam_point = true;
}
#endif

void SquareRootEKFSolver::AddVelocityConstraint(int nRows) {
    static float invSigma = 1.0 / 1e-2;

    static float invRotSigma = 1.0 / 1e-5;
    static float invPosSigma = 1.0 / 1e-2;

#if DISABLE_ACC_BIAS
    int velIdx = nCols - 4;
#else
    int velIdx = 3;
#endif

#if USE_STATIC_STACK
    stacked_matrix_.middleRows(nRows, 9).setZero();
#else
    stacked_matrix_.conservativeResize(nRows + 9, Eigen::NoChange);
    stacked_matrix_.bottomRows<9>().setZero();
#endif
    Matrix3f dR = new_state_->Rwi.transpose() * last_state_->Rwi;
    Vector3f dP = new_state_->Pwi - last_state_->Pwi;

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
    stacked_matrix_.block<3, 3>(nRows, CURRENT_DIM - 2 * CAM_STATE_DIM) =
        Jr * invRotSigma;
    stacked_matrix_.block<3, 3>(nRows, CURRENT_DIM - CAM_STATE_DIM) =
        -Jl * invRotSigma;
#if REMOVE_RESIDUAL_STACK
    obs_residual_.segment(nRows, 3) = -so3 * invRotSigma;
#else
    stacked_matrix_.block<3, 1>(nRows, CURRENT_DIM) = -so3 * invRotSigma;
#endif
    stacked_matrix_.block<3, 3>(nRows + 3,
                                CURRENT_DIM - 2 * CAM_STATE_DIM + 3) =
        -Matrix3f::Identity() * invPosSigma;
    stacked_matrix_.block<3, 3>(nRows + 3, CURRENT_DIM - CAM_STATE_DIM + 3) =
        Matrix3f::Identity() * invPosSigma;
#if REMOVE_RESIDUAL_STACK
    obs_residual_.segment(nRows + 3, 3) = -dP * invPosSigma;
#else
    stacked_matrix_.block<3, 1>(nRows + 3, CURRENT_DIM) = -dP * invPosSigma;
#endif
    stacked_matrix_.block<3, 3>(nRows + 6, velIdx) =
        Matrix3f::Identity() * invSigma;
#if REMOVE_RESIDUAL_STACK
    obs_residual_.segment(nRows + 6, 3) = -*vel_ * invSigma;
#else
    stacked_matrix_.block<3, 1>(nRows + 6, CURRENT_DIM) = -*vel_ * invSigma;
#endif
#if USE_STATIC_STACK
    stacked_rows_ += 9;
#endif
}

#if USE_PLANE_PRIOR
int SquareRootEKFSolver::_AddPlaneContraint() {
    static const float invStd = 1.0 / 1e-5;
    int num_cams = cam_states_.size();
    m_PlaneH.setZero(num_cams, num_cams * 6 + 3 + 1);
    if (!Config::PlaneConstraint) return num_cams;
    int RESIDUAL_IDX = num_cams * 6 + 3;

#if USE_Z_AXIS
    Vector3f theta(m_planeCoeff->x(), m_planeCoeff->y(), 0);
#else
    Vector3f theta(m_planeCoeff->x(), 0, m_planeCoeff->y());
#endif
    float d = m_planeCoeff->z();

    Matrix3f R = Sophus::SO3f::exp(theta).matrix();

    float totalResidual = 0;

    int i = 0;
    for (auto camState : cam_states_) {
        auto& P = camState->Pwi;

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
        m_PlaneH(i, RESIDUAL_IDX) = residual;

        i++;
        totalResidual += residual * residual;
    }

    printf("Residual :%f\n", totalResidual);

    m_PlaneH *= invStd;

    return num_cams;
}
#endif

int SquareRootEKFSolver::StackInformationFactorMatrix() {
    int nTotalObs = 0;
    int nCamStartIdx = IMU_STATE_DIM;
    int nVisualObs = 0;
    int rffIdx = 0;

#if USE_PLANE_PRIOR
    nCamStartIdx += PLANE_DIM;
    nTotalObs += _AddPlaneContraint();
#endif

#if USE_KEYFRAME

    nVisualObs += _AddNewSlamPointConstraint();
    nCamStartIdx += slam_point_.size() * 3;

#endif

#if USE_POSITION_DETECT_ROTATION

    bool addPositionContraint = false;
    if (!nVisualObs && _DetectPureRotation() && !*static_) {
        addPositionContraint = true;
    }
#endif
    int nOldStates = CURRENT_DIM;
    int nCamStates = cam_states_.size() * CAM_STATE_DIM;

    using namespace std;
    for (auto& track : msckf_points_) {
        nVisualObs += track->H.rows();
    }

    nTotalObs += nVisualObs;

#if !USE_STATIC_STACK
    stacked_matrix_.setZero(nTotalObs, nOldStates + 1);
#else
#if REMOVE_RESIDUAL_STACK
    stacked_matrix_.topLeftCorner(nTotalObs, nOldStates).setZero();
    obs_residual_.segment(0, nTotalObs).setZero();
#else
    stacked_matrix_.topLeftCorner(nTotalObs, nOldStates + 1).setZero();
#endif
#endif

#if USE_PLANE_PRIOR

    int num_cams = cam_states_.size();
    stacked_matrix_.topLeftCorner(num_cams, 3) = m_PlaneH.topLeftCorner(num_cams, 3);
    stacked_matrix_.block(0, nCamStartIdx, num_cams, CAM_STATE_DIM * num_cams) =
        m_PlaneH.middleCols(3, CAM_STATE_DIM * num_cams);
#if REMOVE_RESIDUAL_STACK
    obs_residual_.segment(0, num_cams) = m_PlaneH.rightCols<1>();
#else
    stacked_matrix_.block(0, nOldStates, num_cams, 1) = m_PlaneH.rightCols<1>();
#endif

    rffIdx = num_cams;
#endif

    if (!nVisualObs) {
#if USE_STATIC_STACK
#if USE_PLANE_PRIOR
        stacked_rows_ = nTotalObs;
#else
        stacked_rows_ = 0;
#endif
#endif
        return 0;
    }

    float invSigma = 1 / sqrt(Config::ImageNoise2);
    MatrixXf H_j;

    for (auto& track : msckf_points_) {
        int num_obs = track->H.rows();
        track->H *= invSigma;

        stacked_matrix_.block(rffIdx, nCamStartIdx, num_obs, nCamStates) =
            track->H.leftCols(nCamStates);
        //.triangularView<Eigen::Upper>();

#if REMOVE_RESIDUAL_STACK
        obs_residual_.segment(rffIdx, num_obs) = track->H.rightCols<1>();
#else
        stacked_matrix_.block(rffIdx, nOldStates, num_obs, 1) =
            track->H.rightCols<1>();
#endif
        rffIdx += num_obs;
    }

    int slamPointIdx = IMU_STATE_DIM;

#if USE_PLANE_PRIOR
    slamPointIdx += PLANE_DIM;
#endif

#if USE_KEYFRAME

    for (auto& track : slam_point_) {
        if (track->flag_to_marginalize) continue;
        int num_obs = track->H.rows();
        track->H *= invSigma;
        int slamId = track->index_in_window;

        stacked_matrix_.block(rffIdx, nCamStartIdx, num_obs, nCamStates) =
            track->H.middleCols(3, nCamStates);
        stacked_matrix_.block(rffIdx, slamPointIdx + slamId * 3, num_obs, 3) =
            track->H.leftCols<3>();
#if REMOVE_RESIDUAL_STACK
        obs_residual_.segment(rffIdx, num_obs) = track->H.rightCols<1>();
#else
        stacked_matrix_.block(rffIdx, nOldStates, num_obs, 1) =
            track->H.rightCols<1>();
#endif
        rffIdx += num_obs;
    }

#endif

#if USE_POSITION_DETECT_ROTATION

    if (addPositionContraint) {
        nTotalObs += _AddPositionContraint(nTotalObs);
    }

#endif

#if USE_STATIC_STACK
    stacked_rows_ = nTotalObs;
#endif
    return nTotalObs;
}

void SquareRootEKFSolver::SolveAndUpdateStates() {
#if USE_STATIC_STACK
    int haveNewInformation = stacked_rows_;
#else
    int haveNewInformation = stacked_matrix_.rows();
#endif
    if (haveNewInformation) {
#if USE_GIVENS_UPDATE
        TickTock::Start("Givens");
        _UpdateByGivensRotations(haveNewInformation, CURRENT_DIM + 1);
        TickTock::Stop("Givens");
#else

        Eigen::MatrixXf stackedMatrix;
        int nRows = CURRENT_DIM + stacked_matrix_.rows();
        stackedMatrix.setZero(nRows, stacked_matrix_.cols());
        stackedMatrix.topLeftCorner(CURRENT_DIM, CURRENT_DIM) =
            info_factor_matrix_.topLeftCorner(CURRENT_DIM, CURRENT_DIM);
        stackedMatrix.bottomRows(stacked_matrix_.rows()) = stacked_matrix_;

        Eigen::HouseholderQR<MatrixXf> qr(stackedMatrix);
        info_factor_matrix_.topLeftCorner(CURRENT_DIM, CURRENT_DIM) =
            qr.matrixQR()
                .topLeftCorner(CURRENT_DIM, CURRENT_DIM)
                .triangularView<Eigen::Upper>();

        residual_ = qr.matrixQR().topRightCorner(CURRENT_DIM, 1);
#endif

        TickTock::Start("Inverse");

        VectorXf dx = info_factor_matrix_.topLeftCorner(CURRENT_DIM, CURRENT_DIM)
                          .triangularView<Eigen::Upper>()
                          .solve(residual_.segment(0, CURRENT_DIM));
        TickTock::Stop("Inverse");

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
        auto& imuBuffer = ImuBuffer::Instance();
        imuBuffer.UpdateBias(dx.segment<3>(iDim), dx.segment<3>(iDim + 6));
        *vel_ += dx.segment<3>(iDim + 3);
#else
        auto& imuBuffer = ImuBuffer::Instance();
        imuBuffer.UpdateBias(dx.segment<3>(iDim), Vector3f(0, 0, 0));
        *vel_ += dx.segment<3>(iDim + 3);
#endif
        iDim += IMU_STATE_DIM;

#if USE_KEYFRAME
        for (auto& pointState : slam_point_) {
            pointState->Pw += dx.segment<3>(iDim);
            iDim += 3;
        }
#endif

        for (auto& camState : cam_states_) {
            camState->Rwi =
                camState->Rwi *
                Sophus::SO3Group<float>::exp(dx.segment<3>(iDim)).matrix();
            iDim += 3;
            camState->Pwi += dx.segment<3>(iDim);
            iDim += 3;
        }
    }

#if USE_VELOCITY_DETECT_ROTATION
    new_state_->vel = *vel_;
#endif

    msckf_points_.clear();
}

#ifdef PLATFORM_ARM
void SquareRootEKFSolver::_updateByGivensRotationsNeon() {}

#endif

void SquareRootEKFSolver::_UpdateByGivensRotations(int row, int col) {
    for (int j = 0; j < col - 1; ++j) {
        for (int i = row - 1; i >= 0; --i) {
            if (i == 0) {
                float* pI = stacked_matrix_.row(i).data();
                float alpha = info_factor_matrix_(j, j);
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
                        float x = residual_(j);
#if REMOVE_RESIDUAL_STACK
                        float y = obs_residual_(i);
                        obs_residual_(i) = s * x + c * y;
#else
                        float y = pI[k];
                        pI[k] = s * x + c * y;
#endif
                        residual_(j) = c * x + -s * y;

                    } else {
                        float x = info_factor_matrix_(j, k);
                        float y = pI[k];
                        info_factor_matrix_(j, k) = c * x + -s * y;
                        pI[k] = s * x + c * y;
                    }
                }
            } else {
                float* pJ = stacked_matrix_.row(i - 1).data();
                float* pI = stacked_matrix_.row(i).data();
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
                float x = obs_residual_[i - 1];
                float y = obs_residual_[i];
                obs_residual_[i - 1] = c * x + -s * y;
                obs_residual_[i] = s * x + c * y;
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

bool SquareRootEKFSolver::_DetectPureRotation() {
#if USE_POSITION_DETECT_ROTATION

    static std::deque<Eigen::Vector3f> positionDeque;

    positionDeque.push_back(new_state_->Pwi);

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

int SquareRootEKFSolver::_AddPositionContraint(int nRows) {
    static float invPosSigma = 1.0 / 0.05;

#if USE_STATIC_STACK
    stacked_matrix_.middleRows(nRows, 3).setZero();
#else
    stacked_matrix_.conservativeResize(nRows + 3, Eigen::NoChange);
    stacked_matrix_.bottomRows<3>().setZero();
#endif

    Vector3f dP = new_state_->Pwi - last_state_->Pwi;

    stacked_matrix_.block<3, 3>(nRows + 3,
                                CURRENT_DIM - 2 * CAM_STATE_DIM + 3) =
        -Matrix3f::Identity() * invPosSigma;
    stacked_matrix_.block<3, 3>(nRows + 3, CURRENT_DIM - CAM_STATE_DIM + 3) =
        Matrix3f::Identity() * invPosSigma;
#if REMOVE_RESIDUAL_STACK
    obs_residual_.segment(nRows + 3, 3) = -dP * invPosSigma;
#else
    stacked_matrix_.block<3, 1>(nRows + 3, CURRENT_DIM) = -dP * invPosSigma;
#endif
    return 3;
}

#if USE_NEW_MOVED_PIXEL
void SquareRootEKFSolver::_computeDeltaR() {
    for (auto& camState : cam_states_) {
        camState->m_dR.resize(cam_states_.size());
    }

    Matrix3f dR;
    for (int i = 0, n = cam_states_.size(); i < n; ++i) {
        auto camState = cam_states_[i];
        camState->m_dR[i].setIdentity();
        for (int j = i + 1; j < n; ++j) {
            auto camState2 = cam_states_[j];
            dR = camState2->Rwi.transpose() * camState->Rwi;
            camState->m_dR[j] = dR;
            camState2->m_dR[i] = dR.transpose();
        }
    }
}
#endif
void SquareRootEKFSolver::_MarginByGivensRotation() {
    int nRows = CURRENT_DIM;
    for (int j = 0; j < CURRENT_DIM; ++j) {
        for (int i = nRows - 1; i > j; --i) {
            float* pJ = info_factor_matrix_to_marginal_.row(i - 1).data();
            float* pI = info_factor_matrix_to_marginal_.row(i).data();
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

void SquareRootEKFSolver::MarginalizeGivens() {
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

    for (int i = 0, n = slam_point_.size(); i < n; ++i) {
        if (slam_point_[i]->flag_to_marginalize || slam_point_[i]->flag_to_next_marginalize)
            slam_point_[i]->host->flag_dead = true;
        if (slam_point_[i]->host->flag_dead) {
            for (int j = 0; j < 3; ++j) {
                v_MarginDIM.push_back(iDim++);
            }
        } else {
            for (int j = 0; j < 3; ++j) {
                v_RemainDIM.push_back(iDim++);
            }
            v_PointStateNew.push_back(slam_point_[i]);
        }
    }

    slam_point_ = v_PointStateNew;
    for (int i = 0, n = slam_point_.size(); i < n; ++i) {
        slam_point_[i]->index_in_window = i;
    }
#endif

    for (int i = 0, n = cam_states_.size(); i < n; ++i) {
        auto state = cam_states_[i];
        if (state->flag_to_marginalize)
            for (int i = 0; i < CAM_STATE_DIM; i++)
                v_MarginDIM.push_back(iDim++);
        else {
            for (int i = 0; i < CAM_STATE_DIM; i++)
                v_RemainDIM.push_back(iDim++);
            v_CamStateNew.push_back(state);
        }
    }

    cam_states_ = v_CamStateNew;
    cam_states_.push_back(new_state_);
    for (int i = 0, n = cam_states_.size(); i < n; ++i) {
        cam_states_[i]->index_in_window = i;
    }

    for (int i = 0; i < CAM_STATE_DIM; i++) {
        v_RemainDIM.push_back(iDim++);
    }

    // rearrange new information factor matrix

    int index = 0;
    for (auto idx : v_MarginDIM) {
        info_factor_matrix_to_marginal_.col(index++).segment(0, CURRENT_DIM) =
            info_factor_matrix_.col(idx).segment(0, CURRENT_DIM);
    }
    for (auto idx : v_RemainDIM) {
        info_factor_matrix_to_marginal_.col(index++).segment(0, CURRENT_DIM) =
            info_factor_matrix_.col(idx).segment(0, CURRENT_DIM);
    }

    // use qr to marginalize states

    _MarginByGivensRotation();

    int OLD_DIM = CURRENT_DIM;
    CURRENT_DIM = v_RemainDIM.size();

#if USE_KEYFRAME

    info_factor_matrix_after_mariginal_.topLeftCorner(CURRENT_DIM, CURRENT_DIM) =
        info_factor_matrix_to_marginal_.block(OLD_DIM - CURRENT_DIM,
                                           OLD_DIM - CURRENT_DIM, CURRENT_DIM,
                                           CURRENT_DIM);

#else

    info_factor_matrix_.topLeftCorner(CURRENT_DIM, CURRENT_DIM) =
        info_factor_matrix_to_marginal_.block(OLD_DIM - CURRENT_DIM,
                                           OLD_DIM - CURRENT_DIM, CURRENT_DIM,
                                           CURRENT_DIM);

#endif

#if USE_NEW_MOVED_PIXEL

    _computeDeltaR();
#endif
}

void SquareRootEKFSolver::MarginalizeStatic() {
    int iDim = 0;
    std::vector<int> v_MarginDIM, v_RemainDIM;
    std::vector<CamState*> v_CamStateNew;

    for (int i = 0; i < IMU_STATE_DIM; ++i) {
        v_MarginDIM.push_back(i);
        v_RemainDIM.push_back(CURRENT_DIM - IMU_STATE_DIM + i);
    }
    iDim = IMU_STATE_DIM;
    for (int i = 0, n = cam_states_.size(); i < n; ++i) {
        auto state = cam_states_[i];
        if (state->flag_to_marginalize)
            for (int i = 0; i < CAM_STATE_DIM; i++)
                v_MarginDIM.push_back(iDim++);
        else {
            for (int i = 0; i < CAM_STATE_DIM; i++)
                v_RemainDIM.push_back(iDim++);
            v_CamStateNew.push_back(state);
        }
    }
    cam_states_ = v_CamStateNew;
    cam_states_.push_back(new_state_);
    for (int i = 0, n = cam_states_.size(); i < n; ++i) {
        cam_states_[i]->index_in_window = i;
    }

    for (int i = 0; i < CAM_STATE_DIM; i++) {
        v_RemainDIM.push_back(iDim++);
    }

    // rearrange new information factor matrix

    int index = 0;
    for (auto idx : v_MarginDIM) {
        info_factor_matrix_to_marginal_.col(index++) = info_factor_matrix_.col(idx);
    }
    for (auto idx : v_RemainDIM) {
        info_factor_matrix_to_marginal_.col(index++) = info_factor_matrix_.col(idx);
    }

    // use qr to marginalize states

    Eigen::HouseholderQR<MatrixXf> qr(
        info_factor_matrix_to_marginal_.topLeftCorner(CURRENT_DIM, CURRENT_DIM));
    CURRENT_DIM = v_RemainDIM.size();

    info_factor_matrix_.topLeftCorner(CURRENT_DIM, CURRENT_DIM) =
        qr.matrixQR()
            .bottomRightCorner(CURRENT_DIM, CURRENT_DIM)
            .triangularView<Eigen::Upper>();

    // m_infoFactorInverseMatrix = MatrixXf::Identity(CURRENT_DIM, CURRENT_DIM);
    // info_factor_matrix_.topLeftCorner(CURRENT_DIM,
    // CURRENT_DIM).triangularView<Eigen::Upper>().solveInPlace(m_infoFactorInverseMatrix);

    residual_.segment(0, CURRENT_DIM).setZero();
}

}  // namespace DeltaVins
