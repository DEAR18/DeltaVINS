#include "Algorithm/VIOAlgorithm.h"

#include <Eigen/Core>

#include "Algorithm/DataAssociation/DataAssociation.h"
#include "Algorithm/vision/FeatureTrackerOpticalFlow_Chen.h"
#include "IO/dataBuffer/imuBuffer.h"
#include "precompile.h"
#include "utils/TickTock.h"
#include "utils/utils.h"

namespace DeltaVins {
VIOAlgorithm::VIOAlgorithm() {
    feature_trakcer_ = new FeatureTrackerOpticalFlow_Chen(350);
    solver_ = new SquareRootEKFSolver();
    DataAssociation::InitDataAssociation(solver_);
    initialized_ = false;
}

VIOAlgorithm::~VIOAlgorithm() {
    if (feature_trakcer_) {
        delete feature_trakcer_;
        feature_trakcer_ = nullptr;
    }
    if (solver_) {
        delete solver_;
        solver_ = nullptr;
    }
}

void VIOAlgorithm::AddNewFrame(const ImageData::Ptr imageData, Pose::Ptr pose) {
    TickTock::Start("AddFrame");
    // Process input data
    _PreProcess(imageData);

    if (!initialized_) {
        initialized_ = true;
        return;
    }

#if TEST_VISION_MODULE

    _TestVisionModule(imageData, pose);

#else
    TickTock::Start("Propagate");
    // Propagate states
    _AddImuInformation();

    TickTock::Stop("Propagate");

    TickTock::Start("TrackFeature");

    // Track Feature
    feature_trakcer_->MatchNewFrame(states_.tfs_, imageData->image,
                                     frame_now_.get());
    TickTock::Stop("TrackFeature");

    TickTock::Start("Update");

#if USE_KEYFRAME
    _SelectKeyframe();
#endif

    // Update vision measurement
    _AddMeasurement();
    TickTock::Stop("Update");

    TickTock::Stop("AddFrame");

    // Process output data
    _PostProcess(imageData, pose);
#endif
}
void VIOAlgorithm::SetWorldPointAdapter(WorldPointAdapter* adapter) {
    world_point_adapter_ = adapter;
}

void VIOAlgorithm::SetFrameAdapter(FrameAdapter* adapter) {
    frame_adapter_ = adapter;
}
void VIOAlgorithm::_PreProcess(const ImageData::Ptr imageData) {
    auto timestamp = imageData->timestamp;

    frame_now_ = std::make_shared<Frame>();
    frame_now_->timestamp = timestamp;

#if ENABLE_VISUALIZER && !defined(PLATFORM_ARM)
    frame_now_->image = imageData->image.clone();  // Only used for debugging
#endif
    static auto& imuBuffer = ImuBuffer::Instance();
    // Init system
    if (states_.frames_.empty()) {
        // Get Gravity
        Vector3f g = imuBuffer.GetGravity(timestamp);
        LOGI("Gravity:%f %f %f\n", g.x(), g.y(), g.z());
        Matrix3f R = getRotFromGravAndMag(g, Eigen::Vector3f(0, 0, 1));
        imuBuffer.SetZeroBias();
        Initialize(R);
        preintergration_.t0 = timestamp;
        return;
    }
    preintergration_.t1 = timestamp;
    imuBuffer.ImuPreIntegration(preintergration_);

    preintergration_.t0 = preintergration_.t1;
}

void VIOAlgorithm::_PostProcess(ImageData::Ptr data, Pose::Ptr pose) {
    Vector3f Pwi, Vwi;
    Vector3f bg, ba;
    auto* camState = frame_now_->state;
    Matrix3f Rwi = camState->Rwi;

    Pwi = camState->Pwi;
    Vwi = states_.vel;
    pose->timestamp = frame_now_->timestamp;

    pose->Pwb = Pwi * 1e3;
    pose->Rwb = Rwi;

    ImuBuffer::Instance().GetBias(bg, ba);

    Quaternionf _q(Rwi);

    std::string outputName = Config::outputFileName;
    static FILE* file = fopen(outputName.c_str(), "w");
    static FILE* stdvar = fopen("stdvar.csv", "w");

    Vector3f ea = Rwi.transpose().eulerAngles(0, 1, 2);

#ifndef PLATORM_ARM

    fprintf(
        file,
        "%lld,%f,%f,%f,%f,%f,%f,%f,%f,%f,%9.6f,%9.6f,%9.6f,%9.6f,%9.6f,%9.6f\n",
        pose->timestamp, Pwi[0], Pwi[1], -Pwi[2], ea.x(), ea.y(), ea.z(),
        Vwi[0], Vwi[1], Vwi[2], bg[0], bg[1], bg[2], ba[0], ba[1], ba[2]);
#endif
    if (!Config::NoDebugOutput) {
        printf(
            "Timestamp:%lld\n Position:%f,%f,%f\n "
            "Q:%f,%f,%f,%f\nVelocity:%f,%f,%f\n",
            pose->timestamp, Pwi[0], Pwi[1], Pwi[2], _q.w(), _q.x(), _q.y(),
            _q.z(), Vwi[0], Vwi[1], Vwi[2]);
        printf("Gyro Bias:%9.6f,%9.6f,%9.6f\nAcc Bias:%9.6f,%9.6f,%9.6f\n",
               bg[0], bg[1], bg[2], ba[0], ba[1], ba[2]);
    }
    // fflush(file);

#if ENABLE_VISUALIZER || ENABLE_VISUALIZER_TCP || USE_ROS2
    if (!Config::NoGUI) {
        cv::Mat trackImage;
        _DrawTrackImage(data, trackImage);
        if (frame_adapter_) {
            frame_adapter_->PushImageTexture(trackImage.data, trackImage.cols,
                                          trackImage.rows,
                                          trackImage.channels());
            frame_adapter_->FinishFrame();
        } else {
            LOGW("frame_adapter_ is nullptr");
        }

        // cv::imshow("track", trackImage);
        // imshow("FeatureTrack",trackImage);
        // waitKey(1);
    }
#endif
    TickTock::outputResultConsole();
}

void VIOAlgorithm::_UpdatePointsAndCamsToVisualizer() {
#if ENABLE_VISUALIZER || ENABLE_VISUALIZER_TCP||USE_ROS2

    static std::vector<WorldPointGL> vPointsGL;
    static std::vector<FrameGL> vFramesGL;
    vPointsGL.clear();
    vFramesGL.clear();
    vPointsGL.reserve(300);
    vFramesGL.reserve(100);

    static int visCounter = 0;

    for (auto lTrack : states_.tfs_) {
        if (lTrack->point_state_ && lTrack->point_state_->flag_slam_point) {
            if (lTrack->point_state_->m_idVis < 0)
                lTrack->point_state_->m_idVis = visCounter++;
            vPointsGL.emplace_back(lTrack->point_state_->Pw * 1e3,
                                   lTrack->point_state_->m_idVis);
        }
    }

    for (auto frame : states_.frames_) {
        vFramesGL.emplace_back(frame->state->Rwi.matrix(),
                               frame->state->Pwi * 1e3, frame->state->m_id);
    }
    if (!Config::NoGUI) {
        assert(frame_adapter_ && world_point_adapter_);
        frame_adapter_->PushViewMatrix(vFramesGL);
        world_point_adapter_->PushWorldPoint(vPointsGL);
    }
#endif
}

void VIOAlgorithm::_DrawTrackImage(ImageData::Ptr dataPtr,
                                   cv::Mat& trackImage) {
    cvtColor(dataPtr->image, trackImage, cv::COLOR_GRAY2BGR);

    for (auto lTrack : states_.tfs_) {
        if (!lTrack->flag_dead) {
            if (lTrack->point_state_ && lTrack->point_state_->flag_slam_point)
                lTrack->DrawFeatureTrack(trackImage, _GREEN_SCALAR);
            else if (lTrack->num_obs > 5)
                lTrack->DrawFeatureTrack(trackImage, _BLUE_SCALAR);
            else
                lTrack->DrawFeatureTrack(trackImage, _RED_SCALAR);
        }
    }
}

void VIOAlgorithm::_DrawPredictImage(ImageData::Ptr dataPtr,
                                     cv::Mat& predictImage) {
    static std::ofstream fout("Predict.txt");
    cvtColor(dataPtr->image, predictImage, cv::COLOR_GRAY2BGR);

    for (auto lTrack : states_.tfs_) {
        if (!lTrack->flag_dead) {
            if (lTrack->visual_obs.size() >= 2) {
                int nSize = lTrack->visual_obs.size();
                cv::line(predictImage,
                         cv::Point(lTrack->visual_obs[nSize - 2].px.x(),
                                   lTrack->visual_obs[nSize - 2].px.y()),
                         cv::Point(lTrack->visual_obs[nSize - 1].px.x(),
                                   lTrack->visual_obs[nSize - 1].px.y()),
                         _GREEN_SCALAR);
                cv::line(predictImage,
                         cv::Point(lTrack->visual_obs[nSize - 2].px.x(),
                                   lTrack->visual_obs[nSize - 2].px.y()),
                         cv::Point(lTrack->predicted_px.x(),
                                   lTrack->predicted_px.y()),
                         _BLUE_SCALAR);
                cv::circle(predictImage,
                           cv::Point(lTrack->visual_obs[nSize - 2].px.x(),
                                     lTrack->visual_obs[nSize - 2].px.y()),
                           2, _RED_SCALAR);
                cv::circle(predictImage,
                           cv::Point(lTrack->visual_obs[nSize - 1].px.x(),
                                     lTrack->visual_obs[nSize - 1].px.y()),
                           2, _GREEN_SCALAR);
                cv::circle(predictImage,
                           cv::Point(lTrack->predicted_px.x(),
                                     lTrack->predicted_px.y()),
                           2, _BLUE_SCALAR);

                fout << lTrack->visual_obs[nSize - 1].px.x() -
                            lTrack->visual_obs[nSize - 2].px.x()
                     << " "
                     << lTrack->visual_obs[nSize - 1].px.y() -
                            lTrack->visual_obs[nSize - 2].px.y()
                     << " "
                     << lTrack->predicted_px.x() -
                            lTrack->visual_obs[nSize - 2].px.x()
                     << " "
                     << lTrack->predicted_px.x() -
                            lTrack->visual_obs[nSize - 2].px.x()
                     << std::endl;
            }
        }
    }
}

void VIOAlgorithm::Initialize(const Matrix3f& Rwi) {
    auto* camState = frame_now_->state;

    camState->Rwi = Rwi;
    camState->Pwi.setZero();
    camState->Pw_FEJ.setZero();
    camState->index_in_window = 0;
    states_.frames_.push_back(frame_now_);
    states_.vel.setZero();
    states_.static_ = false;
#if USE_PLANE_PRIOR
#if USE_Z_AXIS
    states_.n = Vector3f(0, 0, 1);
#else
    states_.n = Vector3f(0, 1, 0);
#endif

    states_.m_PlaneCoeff.setZero();
    solver_->init(frame_now_->state, &states_.vel, &states_.m_PlaneCoeff,
                    states_.n, &states_.static_);
#else
    solver_->Init(frame_now_->state, &states_.vel, &states_.static_);
#endif
}

void VIOAlgorithm::_AddImuInformation() {
    solver_->AddCamState(frame_now_->state);

    solver_->PropagateStatic(&preintergration_);
#if 0
		solver_->PropagateNew(&preintergration_);
#endif
}

void VIOAlgorithm::_RemoveDeadFeatures() {
    states_.tfs_.remove_if([](const TrackedFeature::Ptr& tracked_feature) {
        return tracked_feature->flag_dead;
    });
}

void VIOAlgorithm::_AddMeasurement() {
    _DetectStill();

    TickTock::Start("Margin");
    _MarginFrames();
    TickTock::Stop("Margin");

    if (states_.tfs_.empty()) return;

    TickTock::Start("DataAssociation");
    DataAssociation::DoDataAssociation(states_.tfs_, states_.static_);

    TickTock::Stop("DataAssociation");
#if ENABLE_VISUALIZER && !defined(PLATFORM_ARM)
#if USE_KEYFRAME
    DataAssociation::DrawPointsBeforeUpdates(solver_->slam_point_);
#else
    std::vector<PointState*> a;
    DataAssociation::DrawPointsBeforeUpdates(a);
#endif

#endif

    TickTock::Start("Stack");
    _StackInformationFactorMatrix();

    TickTock::Stop("Stack");

    TickTock::Start("Solve");
    solver_->SolveAndUpdateStates();
    TickTock::Stop("Solve");
#if ENABLE_VISUALIZER && !defined(PLATFORM_ARM)
#if USE_KEYFRAME
    DataAssociation::DrawPointsAfterUpdates(solver_->slam_point_);

#else
    DataAssociation::DrawPointsAfterUpdates(a);

#endif
    if (!Config::NoGUI) cv::waitKey(5);
#endif
#if ENABLE_VISUALIZER_TCP || ENABLE_VISUALIZER || USE_ROS2
    if (!Config::NoGUI) _UpdatePointsAndCamsToVisualizer();
#endif
    _RemoveDeadFeatures();
}

void VIOAlgorithm::_SelectFrames2Margin() {
    int nCams = states_.frames_.size();
    int cnt = 0;
#if USE_KEYFRAME
    int nKF = 0;
#endif
    for (auto& frame : states_.frames_) {
#if USE_KEYFRAME
        if (frame->flag_keyframe) {
            nKF++;
        }

#endif

        if (frame->tracked_features.empty()) {
            cnt++;
            frame->RemoveAllFeatures();
            frame->state->flag_to_marginalize = true;
        }
    }
    if (!cnt && nCams >= MAX_WINDOW_SIZE) {
        static int camIdxToMargin = 0;
        camIdxToMargin += CAM_DELETE_STEP;
        if (camIdxToMargin >= nCams - 1) camIdxToMargin = 1;
#if USE_KEYFRAME
        if (nKF > 4) {
            for (int i = 0, j = 0; i < nCams; ++i) {
                if (states_.frames_[i]->flag_keyframe) {
                    if (j) {
                        camIdxToMargin = i;
                        break;
                    }
                    j++;
                }
            }
        } else {
            while (camIdxToMargin < nCams &&
                   states_.frames_[camIdxToMargin]->flag_keyframe) {
                camIdxToMargin++;
            }
            if (camIdxToMargin >= nCams - 1) camIdxToMargin = 1;
        }
#endif
        states_.frames_[camIdxToMargin]->RemoveAllFeatures();
        states_.frames_[camIdxToMargin]->state->flag_to_marginalize = true;
    }
}

#if USE_KEYFRAME
void VIOAlgorithm::_SelectKeyframe() {
    if (states_.tfs_.empty()) return;

    auto setkeyframe = [&]() {
        last_keyframe_->flag_keyframe = true;
        for (auto& point : states_.tfs_) {
            if (!point->host_frame)
                point->host_frame = last_keyframe_.get();
        }
    };

    if (last_keyframe_ == nullptr) {
        last_keyframe_ = frame_now_;
        setkeyframe();
        return;
    }

    float nLastKeyframePoints = last_keyframe_->tracked_features.size();

    float nPointsNow = frame_now_->tracked_features.size();

    if (nPointsNow == 0) {
        last_keyframe_ = nullptr;
        return;
    }

    if (nLastKeyframePoints / nPointsNow < 0.6) {
        last_keyframe_ = frame_now_;
        setkeyframe();
    }
}
#endif
void VIOAlgorithm::_MarginFrames() {
    std::vector<Frame::Ptr> vCamStatesNew;

    _SelectFrames2Margin();

#if USE_GIVENS_MARGIN
    solver_->MarginalizeGivens();
#else
    solver_->MarginalizeStatic();
#endif

    for (auto frame : states_.frames_)
        if (!frame->state->flag_to_marginalize) vCamStatesNew.push_back(frame);
    states_.frames_ = vCamStatesNew;
    states_.frames_.push_back(frame_now_);
}

void VIOAlgorithm::_StackInformationFactorMatrix() {
    int nDIM = solver_->StackInformationFactorMatrix();
    if (!nDIM) {
        if (states_.static_) {
            solver_->AddVelocityConstraint(nDIM);
        }
    }
}

bool VIOAlgorithm::_VisionStatic() {
#if USE_NEW_STATIC_DETECT

    int nPxStatic = 0;
    int nAllPx = 0;
    float ratioThresh = 0.3;

    float pxThresh = 0.5 * 0.5;

    for (auto ftTrack : states_.tfs_) {
        if (ftTrack->visual_obs.size() >= 2) {
            if (ftTrack->last_moved_px < pxThresh) nPxStatic++;
            nAllPx++;
        }
    }

    if (nAllPx == 0) return false;

    if (float(nPxStatic) / float(nAllPx) > ratioThresh) return true;
    return false;

#else
    float max_parallax = 0;
    int n = 0;
    for (auto ftTrack : states_.tfs_) {
        if (ftTrack->visual_obs.size() >= 2) {
            max_parallax += sqrt(ftTrack->last_moved_px);
            ++n;
        }
    }
    float lastPx = n ? max_parallax / n : 5;
    static int nFrames = 0;
    static int nStaticFrames = 0;
    static int nMoveFrames = 0;
    float pxThres = 0.5;
    nFrames++;
    if (nFrames < 10) pxThres = 2;

    if (lastPx < pxThres) {
        nStaticFrames++;
        nMoveFrames = 0;
        if (nStaticFrames >= 5) {
            return true;
        }
    } else {
        nStaticFrames = 0;
        nMoveFrames++;
    }
    if (nMoveFrames >= 3) return false;
#endif
}

void VIOAlgorithm::_DetectStill() {
#if USE_NEW_STATIC_DETECT
    static auto& buffer = ImuBuffer::Instance();
    bool bStatic = buffer.DetectStatic(frame_now_->timestamp);

    if (bStatic) {
        if (_VisionStatic()) {
            states_.static_ = true;
            return;
        }
    }

    states_.static_ = false;
#else

    states_.bStatic = _VisionStatic();
#endif
}

void VIOAlgorithm::_TestVisionModule(const ImageData::Ptr data,
                                     Pose::Ptr pose) {
    _AddImuInformation();
    _MarginFrames();
    if (!feature_trakcer_)
        feature_trakcer_ = new FeatureTrackerOpticalFlow_Chen(350);

    feature_trakcer_->MatchNewFrame(states_.tfs_, data->image,
                                     frame_now_.get());

    states_.tfs_.remove_if(
        [](TrackedFeature::Ptr& lf) { return lf->flag_dead; });

    LOGI("%d Point remain", states_.tfs_.size());

#if ENABLE_VISUALIZER
    cv::Mat trackImage;
    cv::Mat PredictImage;
    _DrawTrackImage(data, trackImage);
    _DrawPredictImage(data, PredictImage);
    cv::imshow("Predict", PredictImage);
    cv::waitKey(0);
#endif
    _PostProcess(data, pose);
}

}  // namespace DeltaVins
