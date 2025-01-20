#include "Algorithm/DataAssociation/DataAssociation.h"

#include "Algorithm/DataAssociation/TwoPointRansac.h"
#include "Algorithm/solver/SquareRootEKFSolver.h"
#include "Algorithm/vision/camModel/camModel.h"
#include "dataStructure/vioStructures.h"
#include "precompile.h"
#include "utils/utils.h"

#define MAX_MSCKF_FEATURE_UPDATE_PER_FRAME MAX_POINT_SIZE

namespace DeltaVins {
namespace DataAssociation {

TwoPointRansac* g_two_point_ransac = nullptr;
SquareRootEKFSolver* g_square_root_solver = nullptr;
std::vector<TrackedFeaturePtr> g_tracked_feature_to_update;
std::vector<TrackedFeaturePtr> g_tracked_feature_next_update;
std::vector<std::vector<TrackedFeaturePtr>> g_grid22;

cv::Mat reprojImage;
cv::Mat reprojImage2;

void Clear() {
    g_tracked_feature_next_update.clear();
    g_tracked_feature_to_update.clear();
    delete g_two_point_ransac;
}

void DrawPointsAfterUpdates(std::vector<PointState*>& m_PointStates) {
    if (Config::NoGUI) return;
    reprojImage2 = cv::Mat::zeros(480, 640, CV_8UC3);
    for (auto& p : g_tracked_feature_to_update) {
        p->Reproject();
        for (auto& ob : p->visual_obs) {
            cv::circle(reprojImage2, cv::Point(ob.px.x(), ob.px.y()), 4,
                       _BLUE_SCALAR);
            cv::circle(reprojImage2,
                       cv::Point(ob.px_reprj.x(), ob.px_reprj.y()), 4,
                       _RED_SCALAR);
            cv::line(reprojImage2, cv::Point(ob.px.x(), ob.px.y()),
                     cv::Point(ob.px_reprj.x(), ob.px_reprj.y()),
                     _GREEN_SCALAR);
        }
        for (int i = 0; i < p->visual_obs.size() - 1; ++i) {
            cv::line(reprojImage2,
                     cv::Point(p->visual_obs[i].px.x(),
                               p->visual_obs[i].px.y()),
                     cv::Point(p->visual_obs[i + 1].px.x(),
                               p->visual_obs[i + 1].px.y()),
                     _GREEN_SCALAR);
        }
    }
    for (auto& p2 : m_PointStates) {
        auto p = p2->host;
        p->Reproject();
        auto& ob = p->visual_obs.back();
        cv::circle(reprojImage2, cv::Point(ob.px.x(), ob.px.y()), 10,
                   _BLUE_SCALAR);
        cv::circle(reprojImage2,
                   cv::Point(ob.px_reprj.x(), ob.px_reprj.y()), 10,
                   _RED_SCALAR);
        cv::line(reprojImage2, cv::Point(ob.px.x(), ob.px.y()),
                 cv::Point(ob.px_reprj.x(), ob.px_reprj.y()),
                 _GREEN_SCALAR);
    }
    cv::imshow("Points After Updates", reprojImage2);
}
void DrawPointsBeforeUpdates(std::vector<PointState*>& m_PointStates) {
    if (Config::NoGUI) return;
    reprojImage = cv::Mat::zeros(480, 640, CV_8UC3);

    for (auto& p : g_tracked_feature_to_update) {
        p->Reproject();
        for (auto& ob : p->visual_obs) {
            cv::circle(reprojImage, cv::Point(ob.px.x(), ob.px.y()), 4,
                       _BLUE_SCALAR);
            cv::circle(reprojImage,
                       cv::Point(ob.px_reprj.x(), ob.px_reprj.y()), 4,
                       _RED_SCALAR);
            cv::line(reprojImage, cv::Point(ob.px.x(), ob.px.y()),
                     cv::Point(ob.px_reprj.x(), ob.px_reprj.y()),
                     _GREEN_SCALAR);
        }
        for (int i = 0; i < p->visual_obs.size() - 1; ++i) {
            cv::line(reprojImage,
                     cv::Point(p->visual_obs[i].px.x(),
                               p->visual_obs[i].px.y()),
                     cv::Point(p->visual_obs[i + 1].px.x(),
                               p->visual_obs[i + 1].px.y()),
                     _GREEN_SCALAR);
        }
    }

    for (auto& p2 : m_PointStates) {
        auto p = p2->host;
        p->Reproject();
        auto& ob = p->visual_obs.back();
        cv::circle(reprojImage, cv::Point(ob.px.x(), ob.px.y()), 10,
                   _BLUE_SCALAR);
        cv::circle(reprojImage, cv::Point(ob.px_reprj.x(), ob.px_reprj.y()),
                   10, _RED_SCALAR);
        cv::line(reprojImage, cv::Point(ob.px.x(), ob.px.y()),
                 cv::Point(ob.px_reprj.x(), ob.px_reprj.y()),
                 _GREEN_SCALAR);
    }
    cv::imshow("Points Before Updates", reprojImage);
}

void InitDataAssociation(SquareRootEKFSolver* solver) {
    if (!g_two_point_ransac) g_two_point_ransac = new TwoPointRansac();
    g_square_root_solver = solver;
    g_grid22.resize(4);
}

int RemoveOutlierBy2PointRansac(
    Matrix3f& dR, std::list<TrackedFeaturePtr>& vTrackedFeatures) {
    assert(g_two_point_ransac);

    std::vector<Vector3f> ray0, ray1;
    std::vector<Vector2f> p0, p1;
    std::vector<TrackedFeature*> goodTracks;
    ray0.reserve(600);
    ray1.reserve(600);
    p0.reserve(600);
    p1.reserve(600);
    goodTracks.reserve(600);
    int nGoodPoints = 0;
#if NEW_TWO_POINT
    std::unordered_map<Frame*,
                       std::pair<Matrix3f, std::vector<TrackedFeaturePtr>>>
        vRealtiveSet;

    auto t = std::find_if(vTrackedFeatures.begin(), vTrackedFeatures.end(),
                          [](auto& a) { return !a->flag_dead; });
    Frame* frame = (*t)->visual_obs.back().link_frame;

    for (const auto& trackedFeature : vTrackedFeatures) {
        Frame* frame0 = trackedFeature->visual_obs.front().link_frame;
        if (frame0 != frame) {
            if (!vRealtiveSet.count(frame0))
                vRealtiveSet[frame0].first =
                    frame->state->Rwi.transpose() * frame0->state->Rwi;
            vRealtiveSet[frame0].second.push_back(trackedFeature);
        }
    }

    for (auto& pair : vRealtiveSet) {
        ray0.clear();
        ray1.clear();
        p0.clear();
        p1.clear();
        goodTracks.clear();
        for (const auto& tracked_feature : pair.second.second) {
            if (tracked_feature->flag_dead) continue;
            int nObs = tracked_feature->visual_obs.size();
            auto& lastOb = tracked_feature->visual_obs[nObs - 1];
            ray1.push_back(lastOb.ray);
            p1.push_back(lastOb.px);
            auto& lastSecondOb = tracked_feature->visual_obs[nObs - 2];
            ray0.push_back(lastSecondOb.ray);
            p0.push_back(lastSecondOb.px);
            goodTracks.push_back(tracked_feature.get());
        }

        std::vector<bool> vInliers;
        g_two_point_ransac->FindInliers(p0, ray0, p1, ray1, pair.second.first,
                                       vInliers);
        for (int i = 0, n = vInliers.size(); i < n; ++i) {
            if (!vInliers[i]) {
                auto& track = goodTracks[i];

                track->PopObservation();
                track->flag_dead = true;
            }
        }
    }

#else
    for (const auto& tracked_feature : vTrackedFeatures) {
        if (tracked_feature->flag_dead) continue;
        int nObs = tracked_feature->visual_obs.size();
        auto& lastOb = tracked_feature->visual_obs[nObs - 1];
        ray1.push_back(lastOb.ray);
        p1.push_back(lastOb.px);
        auto& lastSecondOb = tracked_feature->visual_obs[nObs - 2];
        ray0.push_back(lastSecondOb.ray);
        p0.push_back(lastSecondOb.px);
        goodTracks.push_back(tracked_feature.get());
    }

    std::vector<bool> vInliers;
    g_two_point_ransac->FindInliers(p0, ray0, p1, ray1, dR, vInliers);
    for (int i = 0, n = vInliers.size(); i < n; ++i) {
        if (!vInliers[i]) {
            auto& track = goodTracks[i];

            track->flag_dead_frame_id =
                track->visual_obs.back().link_frame->state->m_id;
            track->PopObservation();
            track->flag_dead = true;
            continue;
        }
        nGoodPoints++;
    }
#endif
    return nGoodPoints;
}

void _addBufferPoints(
    std::vector<std::shared_ptr<TrackedFeature>>& vDeadFeature) {
    constexpr int MAX_BUFFER_OBS = 5;
    for (auto trackedFeature : g_tracked_feature_next_update) {
#if USE_KEYFRAME
        if (trackedFeature->point_state_)
            assert(!trackedFeature->point_state_->flag_slam_point);
#endif
        if (trackedFeature->visual_obs.size() > MAX_BUFFER_OBS)
            vDeadFeature.push_back(trackedFeature);
        else
            trackedFeature->RemoveLinksInCamStates();
    }
#if OUTPUT_DEBUG_INFO
    printf("  Add Buffer Points:%d/%d\n", vDeadFeature.size(),
           g_tracked_feature_next_update.size());
#endif
    g_tracked_feature_next_update.clear();
}

void _addDeadPoints(
    std::list<TrackedFeaturePtr>& vTrackedFeatures,
    std::vector<std::shared_ptr<TrackedFeature>>& vDeadFeature) {
    constexpr int MIN_OBS = 4;
    int nDeadPoints2Updates = 0;
    int nDeadPointsAbandoned = 0;
    int nAlivePoints2Updates = 0;
#if USE_KEYFRAME
    constexpr int MIN_OBS_ALIVE = 6;
    constexpr int MIN_OBS_TRACKED = 6;

#else
    constexpr int MIN_OBS_ALIVE = 8;
    constexpr int MIN_OBS_TRACKED = 10;
#endif
    for (auto iter = vTrackedFeatures.begin();
         iter != vTrackedFeatures.end();) {
        auto tracked_feature = *iter;

#if USE_KEYFRAME
        if (tracked_feature->point_state_ &&
            tracked_feature->point_state_->flag_slam_point) {
            ++iter;
            continue;
        }
#endif

        if (tracked_feature->flag_dead) {
            if (tracked_feature->num_obs >= MIN_OBS_TRACKED &&
                tracked_feature->visual_obs.size() >= MIN_OBS) {
                vDeadFeature.push_back(tracked_feature);
                nDeadPoints2Updates++;
            } else {
                tracked_feature->RemoveLinksInCamStates();
                nDeadPointsAbandoned++;
            }
            iter = vTrackedFeatures.erase(iter);
            continue;
        }
#if USE_KEYFRAME
        if (tracked_feature->visual_obs.size() >= MIN_OBS_ALIVE &&
            tracked_feature->num_obs > MIN_OBS_TRACKED) {
#else
        if (tracked_feature->visual_obs.size() >= MIN_OBS_ALIVE) {
#endif
            nAlivePoints2Updates++;
            vDeadFeature.push_back(tracked_feature);
        }
        ++iter;
    }

#if OUTPUT_DEBUG_INFO
    printf("  Dead Points: %d Good / %d Bad\n", nDeadPoints2Updates,
           nDeadPointsAbandoned);
    printf("  AlivePoints to Update:%d\n", nAlivePoints2Updates);
#endif
}

void _pushPoints2Grid(
    const std::vector<std::shared_ptr<TrackedFeature>>& vDeadFeature) {
    static std::vector<std::vector<TrackedFeaturePtr>> vvGrid44(4 * 4);
    static CamModel* camModel = CamModel::getCamModel();
    static const int STEPX = camModel->width() / 4;
    static const int STEPY = camModel->height() / 4;

    auto comparator_less = [](const TrackedFeaturePtr& a,
                              const TrackedFeaturePtr& b) {
        return a->flag_dead == b->flag_dead ? a->ray_angle < b->ray_angle
                                        : a->flag_dead < b->flag_dead;
    };

    auto selectTop2 = [&](const std::vector<TrackedFeaturePtr>& src,
                          std::vector<TrackedFeaturePtr>& dst) {
        TrackedFeaturePtr pFirst = nullptr, pSecond = nullptr;
        if (!src.empty()) {
            for (auto tracked_feature : src) {
                if (!pSecond || comparator_less(pSecond, tracked_feature)) {
                    if (pSecond && pSecond->flag_dead)
                        g_tracked_feature_next_update.push_back(pSecond);
                    pSecond = tracked_feature;
                    if (!pFirst || comparator_less(pFirst, pSecond)) {
                        std::swap(pFirst, pSecond);
                    }
                } else {
                    if (tracked_feature->flag_dead)
                        g_tracked_feature_next_update.push_back(tracked_feature);
                }
            }
            if (pSecond) dst.push_back(pSecond);
            if (pFirst && pFirst != pSecond) dst.push_back(pFirst);
        }
    };

    for (auto deadFeature : vDeadFeature) {
        auto& ob = deadFeature->visual_obs.back();
        vvGrid44[int(ob.px.x() / STEPX) + 4 * int(ob.px.y() / STEPY)]
            .push_back(deadFeature);
    }
#if OUTPUT_DEBUG_INFO
    printf("# 4*4 Dead Points:\n");
    printf("____________________\n");
    printf("|%03d|%03d|%03d|%03d|\n", vvGrid44[0].size(), vvGrid44[1].size(),
           vvGrid44[2].size(), vvGrid44[3].size());
    printf("|%03d|%03d|%03d|%03d|\n", vvGrid44[4].size(), vvGrid44[5].size(),
           vvGrid44[6].size(), vvGrid44[7].size());
    printf("|%03d|%03d|%03d|%03d|\n", vvGrid44[8].size(), vvGrid44[9].size(),
           vvGrid44[10].size(), vvGrid44[11].size());
    printf("|%03d|%03d|%03d|%03d|\n", vvGrid44[12].size(), vvGrid44[13].size(),
           vvGrid44[14].size(), vvGrid44[15].size());
    printf("____________________\n");
#endif

    static int LUT[16] = {0, 0, 1, 1, 0, 0, 1, 1, 2, 2, 3, 3, 2, 2, 3, 3};

    for (int i = 0; i < 16; ++i) {
        selectTop2(vvGrid44[i], g_grid22[LUT[i]]);
    }

    for (auto& grid : g_grid22) {
        std::sort(grid.begin(), grid.end(), comparator_less);
    }

    for (auto& grid : vvGrid44) grid.clear();

#if OUTPUT_DEBUG_INFO
    printf("# 2*2 Dead Points:\n");
    printf("____________________\n");
    printf("|  %03d  |  %03d  |\n", g_grid22[0].size(), g_grid22[1].size());
    printf("|  %03d  |  %03d  |\n", g_grid22[2].size(), g_grid22[3].size());
    printf("____________________\n");
#endif
}

#if 1
void _tryAddMsckfPoseConstraint(
    const std::list<TrackedFeaturePtr>& lTrackFeatures) {
    int nPointsPerGrid = MAX_MSCKF_FEATURE_UPDATE_PER_FRAME / 4;
    int nPointsLeft = MAX_MSCKF_FEATURE_UPDATE_PER_FRAME;
    int nPointsAllAdded = 0;
    int nPointsTriangleFailed = 0;
    int nPointsMahalaFailed = 0;

#if USE_KEYFRAME
    int halfX = CamModel::getCamModel()->width() / 2;
    int halfY = CamModel::getCamModel()->height() / 2;
    int nPointsSlamPerGrid = MAX_POINT_SIZE / 4;
    std::vector<int> vPointsSLAMLeft{0, 0, 0, 0};
    std::vector<int> vPointsSLAMNow{0, 0, 0, 0};
    static std::vector<std::vector<TrackedFeature*>> m_slamPointGrid22(4);
    int nSlamPoint = 0;

    std::for_each(m_slamPointGrid22.begin(), m_slamPointGrid22.end(),
                  [](auto& a) { a.clear(); });
    for (auto& point : lTrackFeatures) {
        if (point->point_state_ && point->point_state_->flag_slam_point && !point->flag_dead) {
            float x = point->visual_obs.back().px.x();
            float y = point->visual_obs.back().px.y();
            if (x < halfX)
                if (y < halfY) {
                    vPointsSLAMNow[0]++;
                    m_slamPointGrid22[0].push_back(point.get());
                } else {
                    vPointsSLAMNow[2]++;
                    m_slamPointGrid22[2].push_back(point.get());
                }
            else if (y < halfY) {
                vPointsSLAMNow[1]++;
                m_slamPointGrid22[1].push_back(point.get());
            } else {
                m_slamPointGrid22[3].push_back(point.get());
                vPointsSLAMNow[3]++;
            }
            nSlamPoint++;
        }
    }
    if (nSlamPoint < 16) {
        for (int i = nSlamPoint; i < 16; i++) {
            int k = 0;
            for (int j = 0; j < 3; j++) {
                k = vPointsSLAMNow[k] <= vPointsSLAMNow[j + 1] ? k : j + 1;
            }
            vPointsSLAMLeft[k]++;
            vPointsSLAMNow[k]++;
        }
    } else {
        for (int i = 0; i < 4; ++i) {
            if (vPointsSLAMNow[i] > 4) {
                m_slamPointGrid22[i][2]->point_state_->flag_to_next_marginalize = true;
                vPointsSLAMLeft[i]++;
            }
        }
    }

    LOGD("SlamCnt:%d %d %d %d %d", nSlamPoint, vPointsSLAMLeft[0],
         vPointsSLAMLeft[1], vPointsSLAMLeft[2], vPointsSLAMLeft[3]);
    nPointsLeft = (MAX_OBS_SIZE - MAX_ADDITIONAL_POINT * MAX_WINDOW_SIZE * 2 -
                   nSlamPoint * 5) /
                  (MAX_WINDOW_SIZE * 2);
    nPointsPerGrid = nPointsLeft / 4;

#endif

    std::vector<int> vPointsLeft = {nPointsPerGrid, nPointsPerGrid,
                                    nPointsPerGrid, nPointsPerGrid};

#if USE_KEYFRAME

#endif

    auto triangleAndVerify = [&](TrackedFeaturePtr& track) {
        if (track->Triangulate()) {
#if OUTPUT_DEBUG_INFO
            printf("#### Triangulation Success\n");
#endif
            if (g_square_root_solver->ComputeJacobians(track.get())) {
                if (g_square_root_solver->MahalanobisTest(track->point_state_)) {
                    nPointsAllAdded++;
                    return true;
                }
                nPointsMahalaFailed++;
                return false;
            }
            return false;
        }
        nPointsTriangleFailed++;
#if OUTPUT_DEBUG_INFO
        printf("#### Triangulation Fail\n");
#endif
        return false;
    };

    auto selectPoints = [&]() {
#if OUTPUT_DEBUG_INFO
        int ii = 0;
        int jInGrid = 0;

#endif
        for (int i = 0; i < 4; ++i) {
            auto& grid = g_grid22[i];
#if OUTPUT_DEBUG_INFO

            printf("## Grid %d in 2*2:\n", ii);
            ii++;
#endif
            while (vPointsLeft[i] && !grid.empty()) {
#if OUTPUT_DEBUG_INFO
                printf("### %d point in Grid %d\n", jInGrid++, ii);
#endif
                auto& ft = grid.back();
                if (triangleAndVerify(ft)) {
                    --nPointsLeft;
#if OUTPUT_DEBUG_INFO
                    ++nPointsAllAdded;
#endif
#if USE_KEYFRAME
                    if (!ft->flag_dead && ft->flag_slam_point_candidate &&
                        vPointsSLAMLeft[i]) {  // If it is a slam
                                               // point

                        g_square_root_solver->AddSlamPoint(ft->point_state_);
                        vPointsSLAMLeft[i]--;
                    } else {
#endif
                        g_square_root_solver->AddMsckfPoint(ft->point_state_);
                        ft->flag_dead = true;
#if USE_KEYFRAME
                    }
#endif
                    vPointsLeft[i]--;

                    g_tracked_feature_to_update.push_back(ft);
                } else
                    ft->flag_dead = false;
                grid.pop_back();
            }
        }
    };
    auto bufferPoints = [&]() {
        for (auto& grid : g_grid22) {
            for (auto& ft : grid) {
                if (ft->flag_dead) {
                    g_tracked_feature_next_update.push_back(ft);
                }
            }
            grid.clear();
        }
    };

    selectPoints();

    if (nPointsLeft) {
        int nMoreGrid = 0;
        for (auto& ptLeft : vPointsLeft) {
            if (ptLeft == 0) nMoreGrid++;
        }
        for (auto& ptLeft : vPointsLeft) {
            if (ptLeft == 0)
                ptLeft = nPointsLeft / nMoreGrid;
            else
                ptLeft = 0;
        }
        selectPoints();
    }
#if OUTPUT_DEBUG_INFO
    printf("### %d Points to Update\n", nPointsAllAdded);
#endif
    printf(
        "### nPointsAdded:%d nPointsTriangleFailed:%d "
        "nPointsMahalaFailed:%d\n",
        nPointsAllAdded, nPointsTriangleFailed, nPointsMahalaFailed);
    bufferPoints();
}

#endif

#if 0
        void _tryAddMsckfPoseConstraint()
        {
            int nPointsPerGrid = MAX_MSCKF_FEATURE_UPDATE_PER_FRAME / 4;
            int nPointsLeft = MAX_MSCKF_FEATURE_UPDATE_PER_FRAME;
            int nPointsAllAdded = 0;
            int nPointsTriangleFailed = 0;
            int nPointsMahalaFailed = 0;

            auto triangleAndVerify = [&](TrackedFeaturePtr& track)
            {
                if (track->Triangulate())
                {
#if OUTPUT_DEBUG_INFO
                    printf("#### Triangulation Success\n");
#endif
                    if (g_square_root_solver->ComputeJacobians(track)) {

                        if (g_square_root_solver->MahalanobisTest(track->m_pState)) {
                            nPointsAllAdded++;
                        	return true;
                        }
                        nPointsMahalaFailed++;
                        return false;
                    }
                    return false;
                }
                nPointsTriangleFailed++;
#if OUTPUT_DEBUG_INFO
                printf("#### Triangulation Fail\n");
#endif
                return false;
            };
            auto selectPoints = [&]()
            {
#if OUTPUT_DEBUG_INFO
                int ii = 0;
				int jInGrid = 0;

#endif
                for (auto& grid : g_grid22)
                {
#if OUTPUT_DEBUG_INFO

                    printf("## Grid %d in 2*2:\n", ii);
					ii++;
#endif
                    int nPointsAdded = 0;
                    while (nPointsAdded < nPointsPerGrid && !grid.empty())
                    {
#if OUTPUT_DEBUG_INFO
                        printf("### %d point in Grid %d\n", jInGrid++, ii);
#endif
                        auto& ft = grid.back();
                        if (triangleAndVerify(ft))
                        {
                            ++nPointsAdded;
                            --nPointsLeft;
#if OUTPUT_DEBUG_INFO
                            ++nPointsAllAdded;
#endif
                            g_square_root_solver->AddMsckfPoint(ft->m_pState);

                            g_tracked_feature_to_update.push_back(ft);

                            ft->flag_dead = true;

                        }
                        else
                            ft->flag_dead = false;
                        grid.pop_back();
                    }


                }

            };
            auto bufferPoints = [&]()
            {
                for (auto& grid : g_grid22)
                {
                    for (auto& ft : grid)
                    {
                        if (ft->flag_dead)
                        {
                            g_tracked_feature_next_update.push_back(ft);
                        }
                    }
                    grid.clear();
                }
            };

            selectPoints();
            nPointsPerGrid = nPointsLeft / 4;

            if (nPointsPerGrid) {
                selectPoints();
            }
#if OUTPUT_DEBUG_INFO
            printf("### %d Points to Update\n", nPointsAllAdded);
#endif
            printf("### nPointsAdded:%d nPointsTriangleFailed:%d nPointsMahalaFailed:%d\n", nPointsAllAdded, nPointsTriangleFailed, nPointsMahalaFailed);
            bufferPoints();
        }
#endif

void DoDataAssociation(std::list<TrackedFeaturePtr>& vTrackedFeatures,
                       bool static_) {
    g_tracked_feature_to_update.clear();

#if USE_STATIC_DETECTION
    if (static_) {
        g_tracked_feature_next_update.clear();
        return;
    }

#endif
    static std::vector<TrackedFeaturePtr> vDeadFeature;

    _addBufferPoints(vDeadFeature);

    _addDeadPoints(vTrackedFeatures, vDeadFeature);

    _pushPoints2Grid(vDeadFeature);

    vDeadFeature.clear();

    _tryAddMsckfPoseConstraint(vTrackedFeatures);
}

}  // namespace DataAssociation
}  // namespace DeltaVins
