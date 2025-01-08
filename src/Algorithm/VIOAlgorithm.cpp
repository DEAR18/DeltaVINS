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
    mp_featureTracker = new FeatureTrackerOpticalFlow_Chen(350);
    mp_solver = new SquareRootEKFSolver();
    DataAssociation::InitDataAssociation(mp_solver);
    mb_Initialized = false;
}

VIOAlgorithm::~VIOAlgorithm() {
    if (mp_featureTracker) {
        delete mp_featureTracker;
        mp_featureTracker = nullptr;
    }
    if (mp_solver) {
        delete mp_solver;
        mp_solver = nullptr;
    }
}

void VIOAlgorithm::addNewFrame(const ImageData::Ptr imageData, Pose::Ptr pose) {
    TickTock::start("AddFrame");
    // Process input data
    _preProcess(imageData);

    if (!mb_Initialized) {
        mb_Initialized = true;
        return;
    }

#if TEST_VISION_MODULE

    _testVisionModule(imageData, pose);

#else
    TickTock::start("Propagate");
    // Propagate states
    _addImuInformation();

    TickTock::stop("Propagate");

    TickTock::start("TrackFeature");

    // Track Feature
    mp_featureTracker->matchNewFrame(m_states.ml_tfs, imageData->image,
                                     mp_frameNow.get());
    TickTock::stop("TrackFeature");

    TickTock::start("Update");

#if USE_KEYFRAME
    _selectKeyframe();
#endif

    // Update vision measurement
    _addMeasurement();
    TickTock::stop("Update");

    TickTock::stop("AddFrame");

    // Process output data
    _postProcess(imageData, pose);
#endif
}
void VIOAlgorithm::setWorldPointAdapter(WorldPointAdapter* adapter) {
    mp_WorldPointAdapter = adapter;
}

void VIOAlgorithm::setFrameAdapter(FrameAdapter* adapter) {
    mp_FrameAdapter = adapter;
}
void VIOAlgorithm::_preProcess(const ImageData::Ptr imageData) {
    auto timestamp = imageData->timestamp;

    mp_frameNow = std::make_shared<Frame>();
    mp_frameNow->timestamp = timestamp;

#if ENABLE_VISUALIZER && !defined(PLATFORM_ARM)
    mp_frameNow->image = imageData->image.clone();  // Only used for debugging
#endif
    static auto& imuBuffer = ImuBuffer::getInstance();
    // Init system
    if (m_states.mv_frames.empty()) {
        // Get Gravity
        Vector3f g = imuBuffer.getGravity(timestamp);
        LOGI("Gravity:%f %f %f\n", g.x(), g.y(), g.z());
        Matrix3f R = getRotFromGravAndMag(g, Eigen::Vector3f(0, 0, 1));
        imuBuffer.setZeroBias();
        initialize(R);
        m_preintergration.t0 = timestamp;
        return;
    }
    m_preintergration.t1 = timestamp;
    imuBuffer.imuPreIntegration(m_preintergration);

    m_preintergration.t0 = m_preintergration.t1;
}

void VIOAlgorithm::_postProcess(ImageData::Ptr data, Pose::Ptr pose) {
    Vector3f Pwi, Vwi;
    Vector3f bg, ba;
    auto* camState = mp_frameNow->state;
    Matrix3f Rwi = camState->m_Rwi;

    Pwi = camState->m_Pwi;
    Vwi = m_states.vel;
    pose->timestamp = mp_frameNow->timestamp;

    pose->Pwb = Pwi * 1e3;
    pose->Rwb = Rwi;

    ImuBuffer::getInstance().getBias(bg, ba);

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

#if ENABLE_VISUALIZER || ENABLE_VISUALIZER_TCP
    if (!Config::NoGUI) {
        cv::Mat trackImage;
        _drawTrackImage(data, trackImage);
        cv::imshow("track", trackImage);
        assert(mp_FrameAdapter);
        mp_FrameAdapter->pushImageTexture(trackImage.data, trackImage.cols,
                                          trackImage.rows,
                                          trackImage.channels());
        mp_FrameAdapter->finishFrame();
    }
#endif
    // imshow("FeatureTrack",trackImage);
    // waitKey(1);
    TickTock::outputResultConsole();
}

void VIOAlgorithm::_updatePointsAndCamsToVisualizer() {
#if ENABLE_VISUALIZER || ENABLE_VISUALIZER_TCP

    static std::vector<WorldPointGL> vPointsGL;
    static std::vector<FrameGL> vFramesGL;
    vPointsGL.clear();
    vFramesGL.clear();
    vPointsGL.reserve(300);
    vFramesGL.reserve(100);

    static int visCounter = 0;

    for (auto lTrack : m_states.ml_tfs) {
        if (lTrack->m_pState && lTrack->m_pState->bSlamPoint) {
            if (lTrack->m_pState->m_idVis < 0)
                lTrack->m_pState->m_idVis = visCounter++;
            vPointsGL.emplace_back(lTrack->m_pState->m_Pw * 1e3,
                                   lTrack->m_pState->m_idVis);
        }
    }

    for (auto frame : m_states.mv_frames) {
        vFramesGL.emplace_back(frame->state->m_Rwi.matrix(),
                               frame->state->m_Pwi * 1e3, frame->state->m_id);
    }
    if (!Config::NoGUI) {
        assert(mp_FrameAdapter && mp_WorldPointAdapter);
        mp_FrameAdapter->pushViewMatrix(vFramesGL);
        mp_WorldPointAdapter->pushWorldPoint(vPointsGL);
    }
#endif
}

void VIOAlgorithm::_drawTrackImage(ImageData::Ptr dataPtr,
                                   cv::Mat& trackImage) {
    cvtColor(dataPtr->image, trackImage, CV_GRAY2BGR);

    for (auto lTrack : m_states.ml_tfs) {
        if (!lTrack->m_bDead) {
            if (lTrack->m_pState && lTrack->m_pState->bSlamPoint)
                lTrack->drawFeatureTrack(trackImage, _GREEN_SCALAR);
            else if (lTrack->m_nObs > 5)
                lTrack->drawFeatureTrack(trackImage, _BLUE_SCALAR);
            else
                lTrack->drawFeatureTrack(trackImage, _RED_SCALAR);
        }
    }
}

void VIOAlgorithm::_drawPredictImage(ImageData::Ptr dataPtr,
                                     cv::Mat& predictImage) {
    static std::ofstream fout("Predict.txt");
    cvtColor(dataPtr->image, predictImage, CV_GRAY2BGR);

    for (auto lTrack : m_states.ml_tfs) {
        if (!lTrack->m_bDead) {
            if (lTrack->m_vVisualObs.size() >= 2) {
                int nSize = lTrack->m_vVisualObs.size();
                cv::line(predictImage,
                         cv::Point(lTrack->m_vVisualObs[nSize - 2].m_px.x(),
                                   lTrack->m_vVisualObs[nSize - 2].m_px.y()),
                         cv::Point(lTrack->m_vVisualObs[nSize - 1].m_px.x(),
                                   lTrack->m_vVisualObs[nSize - 1].m_px.y()),
                         _GREEN_SCALAR);
                cv::line(predictImage,
                         cv::Point(lTrack->m_vVisualObs[nSize - 2].m_px.x(),
                                   lTrack->m_vVisualObs[nSize - 2].m_px.y()),
                         cv::Point(lTrack->m_PredictedPx.x(),
                                   lTrack->m_PredictedPx.y()),
                         _BLUE_SCALAR);
                cv::circle(predictImage,
                           cv::Point(lTrack->m_vVisualObs[nSize - 2].m_px.x(),
                                     lTrack->m_vVisualObs[nSize - 2].m_px.y()),
                           2, _RED_SCALAR);
                cv::circle(predictImage,
                           cv::Point(lTrack->m_vVisualObs[nSize - 1].m_px.x(),
                                     lTrack->m_vVisualObs[nSize - 1].m_px.y()),
                           2, _GREEN_SCALAR);
                cv::circle(predictImage,
                           cv::Point(lTrack->m_PredictedPx.x(),
                                     lTrack->m_PredictedPx.y()),
                           2, _BLUE_SCALAR);

                fout << lTrack->m_vVisualObs[nSize - 1].m_px.x() -
                            lTrack->m_vVisualObs[nSize - 2].m_px.x()
                     << " "
                     << lTrack->m_vVisualObs[nSize - 1].m_px.y() -
                            lTrack->m_vVisualObs[nSize - 2].m_px.y()
                     << " "
                     << lTrack->m_PredictedPx.x() -
                            lTrack->m_vVisualObs[nSize - 2].m_px.x()
                     << " "
                     << lTrack->m_PredictedPx.x() -
                            lTrack->m_vVisualObs[nSize - 2].m_px.x()
                     << std::endl;
            }
        }
    }
}

void VIOAlgorithm::initialize(const Matrix3f& Rwi) {
    auto* camState = mp_frameNow->state;

    camState->m_Rwi = Rwi;
    camState->m_Pwi.setZero();
    camState->m_Pw_FEJ.setZero();
    camState->m_idx = 0;
    m_states.mv_frames.push_back(mp_frameNow);
    m_states.vel.setZero();
    m_states.bStatic = false;
#if USE_PLANE_PRIOR
#if USE_Z_AXIS
    m_states.n = Vector3f(0, 0, 1);
#else
    m_states.n = Vector3f(0, 1, 0);
#endif

    m_states.m_PlaneCoeff.setZero();
    mp_solver->init(mp_frameNow->state, &m_states.vel, &m_states.m_PlaneCoeff,
                    m_states.n, &m_states.bStatic);
#else
    mp_solver->Init(mp_frameNow->state, &m_states.vel, &m_states.bStatic);
#endif
}

void VIOAlgorithm::_addImuInformation() {
    mp_solver->AddCamState(mp_frameNow->state);

    mp_solver->PropagateStatic(&m_preintergration);
#if 0
		mp_solver->PropagateNew(&m_preintergration);
#endif
}

void VIOAlgorithm::_removeDeadFeatures() {
    m_states.ml_tfs.remove_if([](const TrackedFeature::Ptr& tracked_feature) {
        return tracked_feature->m_bDead;
    });
}

void VIOAlgorithm::_addMeasurement() {
    _DetectStill();

    TickTock::start("Margin");
    _marginFrames();
    TickTock::stop("Margin");

    if (m_states.ml_tfs.empty()) return;

    TickTock::start("DataAssociation");
    DataAssociation::DoDataAssociation(m_states.ml_tfs, m_states.bStatic);

    TickTock::stop("DataAssociation");
#if ENABLE_VISUALIZER && !defined(PLATFORM_ARM)
#if USE_KEYFRAME
    DataAssociation::DrawPointsBeforeUpdates(mp_solver->m_vSlamPoint);
#else
    std::vector<PointState*> a;
    DataAssociation::DrawPointsBeforeUpdates(a);
#endif

#endif

    TickTock::start("Stack");
    _stackInformationFactorMatrix();

    TickTock::stop("Stack");

    TickTock::start("Solve");
    mp_solver->solveAndUpdateStates();
    TickTock::stop("Solve");
#if ENABLE_VISUALIZER && !defined(PLATFORM_ARM)
#if USE_KEYFRAME
    DataAssociation::DrawPointsAfterUpdates(mp_solver->m_vSlamPoint);

#else
    DataAssociation::DrawPointsAfterUpdates(a);

#endif
    if (!Config::NoGUI) cv::waitKey(5);
#endif
#if ENABLE_VISUALIZER_TCP || ENABLE_VISUALIZER
    if (!Config::NoGUI) _updatePointsAndCamsToVisualizer();
#endif
    _removeDeadFeatures();
}

void VIOAlgorithm::_selectFrames2Margin() {
    int nCams = m_states.mv_frames.size();
    int cnt = 0;
#if USE_KEYFRAME
    int nKF = 0;
#endif
    for (auto& frame : m_states.mv_frames) {
#if USE_KEYFRAME
        if (frame->m_bKeyframe) {
            nKF++;
        }

#endif

        if (frame->m_vTrackedFeatures.empty()) {
            cnt++;
            frame->removeAllFeatures();
            frame->state->m_bToMargin = true;
        }
    }
    if (!cnt && nCams >= MAX_WINDOW_SIZE) {
        static int camIdxToMargin = 0;
        camIdxToMargin += CAM_DELETE_STEP;
        if (camIdxToMargin >= nCams - 1) camIdxToMargin = 1;
#if USE_KEYFRAME
        if (nKF > 4) {
            for (int i = 0, j = 0; i < nCams; ++i) {
                if (m_states.mv_frames[i]->m_bKeyframe) {
                    if (j) {
                        camIdxToMargin = i;
                        break;
                    }
                    j++;
                }
            }
        } else {
            while (camIdxToMargin < nCams &&
                   m_states.mv_frames[camIdxToMargin]->m_bKeyframe) {
                camIdxToMargin++;
            }
            if (camIdxToMargin >= nCams - 1) camIdxToMargin = 1;
        }
#endif
        m_states.mv_frames[camIdxToMargin]->removeAllFeatures();
        m_states.mv_frames[camIdxToMargin]->state->m_bToMargin = true;
    }
}

#if USE_KEYFRAME
void VIOAlgorithm::_selectKeyframe() {
    if (m_states.ml_tfs.empty()) return;

    auto setkeyframe = [&]() {
        mp_lastKeyframe->m_bKeyframe = true;
        for (auto& point : m_states.ml_tfs) {
            if (!point->m_pHostFrame)
                point->m_pHostFrame = mp_lastKeyframe.get();
        }
    };

    if (mp_lastKeyframe == nullptr) {
        mp_lastKeyframe = mp_frameNow;
        setkeyframe();
        return;
    }

    float nLastKeyframePoints = mp_lastKeyframe->m_vTrackedFeatures.size();

    float nPointsNow = mp_frameNow->m_vTrackedFeatures.size();

    if (nPointsNow == 0) {
        mp_lastKeyframe = nullptr;
        return;
    }

    if (nLastKeyframePoints / nPointsNow < 0.6) {
        mp_lastKeyframe = mp_frameNow;
        setkeyframe();
    }
}
#endif
void VIOAlgorithm::_marginFrames() {
    std::vector<Frame::Ptr> vCamStatesNew;

    _selectFrames2Margin();

#if USE_GIVENS_MARGIN
    mp_solver->marginalizeGivens();
#else
    mp_solver->marginalizeStatic();
#endif

    for (auto frame : m_states.mv_frames)
        if (!frame->state->m_bToMargin) vCamStatesNew.push_back(frame);
    m_states.mv_frames = vCamStatesNew;
    m_states.mv_frames.push_back(mp_frameNow);
}

void VIOAlgorithm::_stackInformationFactorMatrix() {
    int nDIM = mp_solver->stackInformationFactorMatrix();
    if (!nDIM) {
        if (m_states.bStatic) {
            mp_solver->addVelocityConstraint(nDIM);
        }
    }
}

bool VIOAlgorithm::_visionStatic() {
#if USE_NEW_STATIC_DETECT

    int nPxStatic = 0;
    int nAllPx = 0;
    float ratioThresh = 0.3;

    float pxThresh = 0.5 * 0.5;

    for (auto ftTrack : m_states.ml_tfs) {
        if (ftTrack->m_vVisualObs.size() >= 2) {
            if (ftTrack->m_LastMovedPx < pxThresh) nPxStatic++;
            nAllPx++;
        }
    }

    if (nAllPx == 0) return false;

    if (float(nPxStatic) / float(nAllPx) > ratioThresh) return true;
    return false;

#else
    float max_parallax = 0;
    int n = 0;
    for (auto ftTrack : m_states.ml_tfs) {
        if (ftTrack->m_vVisualObs.size() >= 2) {
            max_parallax += sqrt(ftTrack->m_LastMovedPx);
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
    static auto& buffer = ImuBuffer::getInstance();
    bool bStatic = buffer.detectStatic(mp_frameNow->timestamp);

    if (bStatic) {
        if (_visionStatic()) {
            m_states.bStatic = true;
            return;
        }
    }

    m_states.bStatic = false;
#else

    m_states.bStatic = _visionStatic();
#endif
}

void VIOAlgorithm::_testVisionModule(const ImageData::Ptr data,
                                     Pose::Ptr pose) {
    _addImuInformation();
    _marginFrames();
    if (!mp_featureTracker)
        mp_featureTracker = new FeatureTrackerOpticalFlow_Chen(350);

    mp_featureTracker->matchNewFrame(m_states.ml_tfs, data->image,
                                     mp_frameNow.get());

    m_states.ml_tfs.remove_if(
        [](TrackedFeature::Ptr& lf) { return lf->m_bDead; });

    LOGI("%d Point remain", m_states.ml_tfs.size());

#if ENABLE_VISUALIZER
    cv::Mat trackImage;
    cv::Mat PredictImage;
    _drawTrackImage(data, trackImage);
    _drawPredictImage(data, PredictImage);
    cv::imshow("Predict", PredictImage);
    cv::waitKey(0);
#endif
    _postProcess(data, pose);
}

}  // namespace DeltaVins
