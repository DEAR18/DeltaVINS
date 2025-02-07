#include "dataStructure/vioStructures.h"

#include "Algorithm/vision/camModel/camModel.h"
#include "precompile.h"
#include "utils/utils.h"

namespace DeltaVins {
Frame::Frame() {
    state = new CamState();
    state->host_frame = this;
#if USE_KEYFRAME
    flag_keyframe = false;
#endif
}

Frame::~Frame() {
    if (state) delete state;
}

void Frame::RemoveLinksFromAllTrackedFeatures(TrackedFeature* ftTrack) {
    tracked_features.erase(ftTrack);
}

void Frame::RemoveAllFeatures() {
    for (auto feature : tracked_features) {
#if USE_KEYFRAME
        if (feature->host_frame == this) feature->host_frame = nullptr;
#endif
        for (auto& observation : feature->visual_obs) {
            if (observation.link_frame == this) {
                int nSize = feature->visual_obs.size();
                if (!feature->flag_dead) {
                    observation = feature->visual_obs[nSize - 2];
                    feature->visual_obs[nSize - 2] =
                        feature->visual_obs[nSize - 1];
                }

                else
                    observation = feature->visual_obs.back();
                feature->visual_obs.pop_back();
                break;
            }
        }
    }
    tracked_features.clear();
}

TrackedFeature::~TrackedFeature() {
    if (!visual_obs.empty()) RemoveLinksInCamStates();
    if (point_state_) {
        delete point_state_;
        point_state_ = nullptr;
    }
}

TrackedFeature::TrackedFeature() : NonLinear_LM(1e-2, 0.005, 1e-3, 15, false) {
    flag_dead = false;
    num_obs = 0;
    ray_angle = 0;
    last_moved_px = 0;
    point_state_ = nullptr;
#if USE_KEYFRAME
    flag_slam_point_candidate = 0;
    host_frame = nullptr;
#endif


    static int counter = 0;
    m_id = counter++;
}

bool TrackedFeature::Triangulate() {


    if (verbose_) LOGI("###PointID:%d", m_id);
    // if (point_state_) return m_Result.bConverged;
    static Vector3f Tci = CamModel::getCamModel()->getTci();

    clear();

    auto& leftVisualOb = visual_obs.front();
    // Vector3f pInImu = Rci.transpose() * (leftVisualOb.m_Ray_cam*2) + Pic;
    z = leftVisualOb.ray;
    z /= z[2];
    z[2] = 1 / 2.f;
    /*z.z() = 1/pInImu.z();*/

    const int nSize = visual_obs.size();
    dRs.resize(nSize);
    dts.resize(nSize);
    Matrix3f leftR = leftVisualOb.link_frame->state->Rwi;
    Vector3f leftP = leftVisualOb.link_frame->state->Pwi;
    {
        for (int i = 0; i < nSize; ++i) {
            auto& ob = visual_obs[i];
            Matrix3f R_i = ob.link_frame->state->Rwi;
            Vector3f P_i = ob.link_frame->state->Pwi;
            dRs[i] = R_i.transpose() * leftR;
            dts[i] = R_i.transpose() * (leftP - P_i) + Tci - dRs[i] * Tci;
        }
    }

    solve();

    if (point_state_ == nullptr) {
        point_state_ = new PointState();
        point_state_->host = this;
    }
    Vector3f cpt = z / z[2];
    cpt[2] = 1.0f / z[2];
    point_state_->Pw = leftR * (cpt - Tci) + leftP;
    point_state_->Pw_FEJ = point_state_->Pw;
#if USE_KEYFRAME
    float depthRatio = 0.01;
    if (m_Result.bConverged && !flag_dead && m_Result.cost < 2 &&
        H(2, 2) > depthRatio * H(0, 0) && H(2, 2) > depthRatio * H(1, 1))
        flag_slam_point_candidate = true;

    if (z[2] < 0.1) flag_slam_point_candidate = false;

#endif

    dRs.clear();
    dts.clear();
    return m_Result.bConverged;
}

float TrackedFeature::EvaluateF(bool bNewZ, float huberThresh) {
    float cost = 0.f;
    const int nSize = visual_obs.size();
    Matrix23f J23;
    Matrix3f J33;
    Vector3f position;

    // float huberCutTh = 10.f;

    if (bNewZ) {
        position = zNew / zNew[2];
        position[2] = 1.0 / zNew[2];
    } else {
        position = z / z[2];
        position[2] = 1.0 / z[2];
    }
    HTemp.setZero();
    bTemp.setZero();
    J33 << position[2], 0, -position[0] * position[2], 0, position[2],
        -position[1] * position[2], 0, 0, -position[2] * position[2];
    for (int i = 0; i < nSize; ++i) {
        Vector3f p_cam = dRs[i] * position + dts[i];

        visual_obs[i].px_reprj =
            CamModel::getCamModel()->camToImage(p_cam, J23);

        Vector2f r = visual_obs[i].px - visual_obs[i].px_reprj;
#if HUBER || 1
        float reprojErr = r.norm();
        float hw = reprojErr > huberThresh ? huberThresh / reprojErr : 1;
        // cost += reprojErr>huberThresh ?
        // huberThresh*(2*reprojErr-huberThresh):reprojErr*reprojErr;
        if (reprojErr > huberThresh) {
            cost += huberThresh * (2 * reprojErr - huberThresh);
        } else
            cost += reprojErr * reprojErr;
#else
        float hw = 1.0f;
        cost += r.squaredNorm();
#endif

        Matrix23f J = J23 * dRs[i] * J33;

        HTemp.noalias() += J.transpose() * J * hw;
        bTemp.noalias() += J.transpose() * r * hw;
    }
    // if(!bNewZ)
    //	H += Matrix3f::Identity()*FLT_EPSILON;
    return cost;
}

bool TrackedFeature::UserDefinedDecentFail() { return zNew[2] < 0; }

void TrackedFeature::AddVisualObservation(const Vector2f& px, Frame* frame) {
    assert(CamModel::getCamModel()->inView(px));
    flag_dead = false;

    if (!visual_obs.empty()) {
        last_moved_px = (visual_obs.back().px - px).squaredNorm();
    }
    num_obs++;

#if USE_KEYFRAME

    if (point_state_ && point_state_->flag_slam_point) {
        int n = visual_obs.size();
        if (n > 2) {
            auto& obs_3 = visual_obs[n - 3];
            if (!obs_3.link_frame->flag_keyframe) {
                obs_3.link_frame->tracked_features.erase(this);
                obs_3 = visual_obs[n - 2];
                visual_obs[n - 2] = visual_obs[n - 1];
                visual_obs.pop_back();
            }
        }
    }

#endif

    Vector3f ray0 = CamModel::getCamModel()->imageToImu(px);

    visual_obs.emplace_back(px, ray0, frame);

    auto& rightOb = visual_obs.back();

    frame->tracked_features.insert(this);

    ray_angle0 = ray_angle;
    Vector3f ray1 = rightOb.link_frame->state->Rwi * rightOb.ray;

    float minDot = 2;
    for (size_t i = 0, length = visual_obs.size() - 1; i < length; i++) {
        auto& visualOb = visual_obs[i];
        float dot =
            (visualOb.link_frame->state->Rwi * visualOb.ray).dot(ray1);
        if (dot < minDot) minDot = dot;
    }
    if (minDot > 0 && minDot < 1) {
        ray_angle = std::max(ray_angle, acosf(minDot));
    }
}

void TrackedFeature::DrawFeatureTrack(cv::Mat& image, cv::Scalar color) const {
    for (size_t i = 0; i < visual_obs.size() - 1; ++i) {
        if ((visual_obs[i].px - visual_obs[i + 1].px).squaredNorm() >
            900)
            continue;
        cv::line(image,
                 cv::Point(visual_obs[i].px.x(), visual_obs[i].px.y()),
                 cv::Point(visual_obs[i + 1].px.x(),
                           visual_obs[i + 1].px.y()),
                 _GREEN_SCALAR, 1);
        cv::circle(
            image,
            cv::Point(visual_obs[i].px.x(), visual_obs[i].px.y()), 2,
            color);
    }
    cv::circle(
        image,
        cv::Point(visual_obs.back().px.x(), visual_obs.back().px.y()),
        8, color);
}

void TrackedFeature::Reproject() {
    assert(point_state_);
    auto camModel = CamModel::getCamModel();
    float reprojErr = 0;
    for (auto& ob : visual_obs) {
        ob.px_reprj = camModel->imuToImage(
            ob.link_frame->state->Rwi.transpose() *
            (point_state_->Pw - ob.link_frame->state->Pwi));
        reprojErr += (ob.px_reprj - ob.px).norm();
    }
    reprojErr /= visual_obs.size();
    if (reprojErr > 3) {
        if (reprojErr > 5)
            // DrawObservationsAndReprojection();
            printf("Reproj Err:%f, Triangulation Cost:%f\n", reprojErr,
                   m_Result.cost);
    }
}

void TrackedFeature::DrawObservationsAndReprojection(int time) {
#if ENABLE_VISUALIZER && !defined(PLATFORM_ARM)
    if (Config::NoGUI) return;
    cv::Mat display;
    bool first = 1;
    for (auto& ob : visual_obs) {
        cv::cvtColor(ob.link_frame->image, display, cv::COLOR_GRAY2BGR);
        if (first) {
            cv::circle(display, cv::Point(ob.px.x(), ob.px.y()), 8,
                       _GREEN_SCALAR);
            cv::circle(display, cv::Point(ob.px_reprj.x(), ob.px_reprj.y()),
                       10, _BLUE_SCALAR);
            first = 0;
        } else {
            cv::circle(display, cv::Point(ob.px.x(), ob.px.y()), 4,
                       _GREEN_SCALAR);
            cv::circle(display, cv::Point(ob.px_reprj.x(), ob.px_reprj.y()),
                       6, _BLUE_SCALAR);
        }
        cv::imshow("ob and reproj", display);
        cv::waitKey(time);
    }
#else
    (void)time;
#endif
}

void TrackedFeature::PrintObservations() {
    for (size_t i = 0; i < visual_obs.size(); ++i) {
        LOGI("Idx:%ld,Px: %f %f,Pos:%f %f %f", i, visual_obs[i].px.x(),
             visual_obs[i].px.y(),
             visual_obs[i].link_frame->state->Pwi.x(),
             visual_obs[i].link_frame->state->Pwi.y(),
             visual_obs[i].link_frame->state->Pwi.z());
    }
}
#if USE_KEYFRAME
void TrackedFeature::RemoveUselessObservationForSlamPoint() {
    assert(point_state_ && point_state_->flag_slam_point);
    std::vector<VisualObservation> obs;
    for (int i = 0, n = visual_obs.size(); i < n; ++i) {
        auto& ob = visual_obs[i];
        if (ob.link_frame->flag_keyframe || i >= n - 2) {
            obs.push_back(ob);
        } else {
            ob.link_frame->tracked_features.erase(this);
        }
    }

    visual_obs = obs;
}
#endif
bool TrackedFeature::UserDefinedConvergeCriteria() {
    if (m_Result.cost < 10) m_Result.bConverged = true;
    if (m_Result.cost > 40) m_Result.bConverged = false;
    return true;
}

void TrackedFeature::PrintPositions() {}

void TrackedFeature::PopObservation() {
    visual_obs.back().link_frame->RemoveLinksFromAllTrackedFeatures(this);
    ray_angle = ray_angle0;
    visual_obs.pop_back();
    num_obs--;
}

void TrackedFeature::RemoveLinksInCamStates() {
    for (auto& ob : visual_obs)
        ob.link_frame->RemoveLinksFromAllTrackedFeatures(this);
    visual_obs.clear();
}

}  // namespace DeltaVins
