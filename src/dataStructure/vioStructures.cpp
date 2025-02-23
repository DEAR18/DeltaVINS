#include "dataStructure/vioStructures.h"

#include <set>
#include <unordered_map>

#include "Algorithm/Initializer/Triangulation.h"
#include "Algorithm/vision/camModel/camModel.h"
#include "precompile.h"
#include "utils/SensorConfig.h"
#include "utils/tf.h"
#include "utils/utils.h"

namespace DeltaVins {

VisualObservation::VisualObservation(const Vector2f& px, Frame* frame,
                                     int cam_id)
    : px(px), link_frame(frame), cam_id(cam_id) {
    auto camModel = SensorConfig::Instance().GetCamModel(frame->sensor_id);
    ray_in_cam = camModel->imageToCam(px);
}

Frame::Frame(int sensor_id) {
    state = new CamState();
    state->host_frame = this;
    valid_landmark_num = 0;
    this->sensor_id = sensor_id;
    flag_keyframe = false;
    static std::unordered_map<int, int> frame_id_counter;
    frame_id = frame_id_counter[sensor_id]++;
}

VisualObservation::Ptr Frame::AddVisualObservation(const Vector2f& px,
                                                   int cam_id) {
    auto obs = std::make_shared<VisualObservation>(px, this, cam_id);
    visual_obs[cam_id].insert(obs);
    return obs;
}

Frame::~Frame() {
    if (state) delete state;
}

void Frame::RemoveAllObservations() {
    for (int cam_id = 0; cam_id < 2; cam_id++) {
        for (auto obs : visual_obs[cam_id]) {
            Landmark* landmark = obs->link_landmark;
            if (landmark) {
                landmark->RemoveVisualObservation(obs);
            }
        }
        visual_obs[cam_id].clear();
    }
    valid_landmark_num = 0;
}

void Landmark::RemoveVisualObservation(VisualObservation::Ptr obs) {
    int cam_id = obs->cam_id;
    visual_obs[cam_id].erase(obs);
    obs->link_landmark = nullptr;
    // Todo: stereo observation would be handled twice here
    obs->link_frame->valid_landmark_num--;
    valid_obs_num--;
}

Landmark::~Landmark() {
    if (!visual_obs[0].empty() || !visual_obs[1].empty()) {
        RemoveLinksInCamStates();
    }
    if (point_state_) {
        delete point_state_;
        point_state_ = nullptr;
    }
}

void Landmark::SetDeadFlag(bool dead, int cam_id) {
    if (cam_id == -1) {
        flag_dead_all = dead;
        flag_dead[0] = dead;
        flag_dead[1] = dead;
    } else {
        flag_dead[cam_id] = dead;
    }
}

Landmark::Landmark() : NonLinear_LM(1e-2, 0.005, 1e-3, 15, false) {
    flag_dead[0] = true;
    flag_dead[1] = true;
    flag_dead_all = true;
    num_obs_tracked = 0;
    ray_angle = 0;
    ray_angle0 = 0;
    // last_moved_px = 0;
    point_state_ = nullptr;
    flag_slam_point_candidate = false;
    valid_obs_num = 0;
    host_frame = nullptr;

    flag_dead_frame_id[0] = -1;
    flag_dead_frame_id[1] = -1;
    static int counter = 0;
    landmark_id_ = counter++;
}

bool Landmark::TriangulationAnchorDepth(float& anchor_depth) {
    std::vector<Eigen::Vector3d> ray_in_c;
    std::vector<Eigen::Matrix3d> Rwc;
    std::vector<Eigen::Vector3d> Pc_in_w;

    // Todo: support multi-camera
    CamModel::Ptr camModel = SensorConfig::Instance().GetCamModel(0);
    // static Vector3d Tci = camModel->getTci().cast<double>();
    static Matrix3d Rci[2] = {camModel->getRci(0).cast<double>(),
                              camModel->getRci(1).cast<double>()};
    static Vector3d Pc_in_i[2] = {camModel->getPic(0).cast<double>(),
                                  camModel->getPic(1).cast<double>()};
    int anchor_cam_id = visual_obs[0].size() > 0 ? 0 : 1;
    for (int cam_id = 0; cam_id < 2; cam_id++) {
        for (auto& visualOb : visual_obs[cam_id]) {
            ray_in_c.push_back(visualOb->ray_in_cam.cast<double>());
            Rwc.push_back(visualOb->link_frame->state->Rwi.cast<double>() *
                          Rci[cam_id].transpose());
            Pc_in_w.push_back(visualOb->link_frame->state->Pwi.cast<double>() +
                              visualOb->link_frame->state->Rwi.cast<double>() *
                                  Pc_in_i[cam_id]);
        }
    }
    Eigen::Vector3d Pw_triangulated;
    bool success = DeltaVins::TriangulationAnchorDepth(ray_in_c, Rwc, Pc_in_w,
                                                       Pw_triangulated);
    if (success) {
        Eigen::Vector3d Pw_triangulated_cam =
            Rwc[anchor_cam_id].transpose() *
            (Pw_triangulated - Pc_in_w[anchor_cam_id]);
        anchor_depth = Pw_triangulated_cam.z();
        return true;
    }
    return false;
}
bool Landmark::Triangulate() {
    float anchor_depth = 0;
    if (!TriangulationAnchorDepth(anchor_depth)) return false;
    return TriangulateLM(anchor_depth);
}

bool Landmark::TriangulateLM(float depth_prior) {
    if (verbose_) LOGI("###PointID:%d", landmark_id_);
    // if (point_state_) return m_Result.bConverged;
    // TODO: multi-camera triangulation
    // CamModel::Ptr camModel = SensorConfig::Instance().GetCamModel(0);
    // static Vector3d Tci = camModel->getTci().cast<double>();

    clear();

    // select the anchor observation
    // he anchor observation is selected from the right camera only when the
    // left - hand observation is empty.
    VisualObservation::Ptr anchor_ob = nullptr;
    anchor_ob = visual_obs[0].size() > 0 ? *visual_obs[0].begin()
                                         : *visual_obs[1].begin();
    if (anchor_ob == nullptr) {
        return false;
    }
    int anchor_cam_id = anchor_ob->cam_id;

    // Vector3f pInImu = Rci.transpose() * (leftVisualOb.m_Ray_cam*2) + Pic;
    z = anchor_ob->ray_in_cam.cast<double>();
    z /= z[2];
    z[2] = 1 / depth_prior;
    /*z.z() = 1/pInImu.z();*/

    Matrix3f anchor_R = anchor_ob->link_frame->state->Rwi;
    Vector3f anchor_P = anchor_ob->link_frame->state->Pwi;
    Eigen::Isometry3f T_w_i_anchor = Eigen::Isometry3f::Identity();
    T_w_i_anchor.linear() = anchor_R;
    T_w_i_anchor.translation() = anchor_P;
    Transform<float> T_w_i_anchor_tf =
        Transform<float>(0, "world", "imu0", T_w_i_anchor);
    Transform<float> T_i_c0_tf;
    Tfs<float>::Instance().GetTransform("imu0", "camera0", T_i_c0_tf);
    Transform<float> T_i_c1_tf;
    if (!visual_obs[1].empty()) {
        Tfs<float>::Instance().GetTransform("imu0", "camera1", T_i_c1_tf);
    }
    Transform<float> T_i_c_anchor_tf;
    T_i_c_anchor_tf = anchor_cam_id == 0 ? T_i_c0_tf : T_i_c1_tf;
    auto T_w_c_anchor_tf = T_w_i_anchor_tf * T_i_c_anchor_tf;
    // To compute the relative pose between the anchor frame and the current
    // frame

    // get the relative pose between left and right
    Transform<float> T_c0_c1_tf;
    if (!visual_obs[0].empty() && !visual_obs[1].empty()) {
        Tfs<float>::Instance().GetTransform("camera1", "camera0", T_c0_c1_tf);
    }

    for (int cam_id = 0; cam_id < 2; cam_id++) {
        const int nSize = visual_obs[cam_id].size();
        dRs[cam_id].resize(nSize);
        dts[cam_id].resize(nSize);
        Transform<float> T_i_c_n_tf = cam_id == 0 ? T_i_c0_tf : T_i_c1_tf;
        {
            int i = 0;
            for (auto& visualOb : visual_obs[cam_id]) {
                Eigen::Isometry3f T_w_i_n;

                T_w_i_n.linear() = visualOb->link_frame->state->Rwi;
                T_w_i_n.translation() = visualOb->link_frame->state->Pwi;
                Transform<float> T_w_i_n_tf =
                    Transform<float>(0, "world", "imu0", T_w_i_n);

                Transform<float> T_i_n_i_a_tf =
                    T_w_i_n_tf.Inverse() * T_w_i_anchor_tf;
                Transform<float> T_cn_ca_tf =
                    T_i_c_n_tf.Inverse() * T_i_n_i_a_tf * T_i_c_anchor_tf;
                dRs[cam_id][i] = T_cn_ca_tf.Rotation().cast<double>();
                dts[cam_id][i] = T_cn_ca_tf.Translation().cast<double>();
                i++;
            }
        }
    }

    solve();

    if (point_state_ == nullptr) {
        point_state_ = new PointState();
        point_state_->host = this;
    }
    Vector3d cpt = z / z[2];
    cpt[2] = 1.0 / z[2];
    point_state_->Pw = T_w_c_anchor_tf.TransformPoint(cpt.cast<float>());
    point_state_->Pw_FEJ = point_state_->Pw;
    float depthRatio = 0.01;
    if (m_Result.bConverged && !flag_dead && m_Result.cost < 2 &&
        H(2, 2) > depthRatio * H(0, 0) && H(2, 2) > depthRatio * H(1, 1))
        flag_slam_point_candidate = true;

    // if (z[2] < 0.1) flag_slam_point_candidate = false;

    for (int cam_id = 0; cam_id < 2; cam_id++) {
        dRs[cam_id].clear();
        dts[cam_id].clear();
    }

    m_Result.cost = Reproject(false);
    if (m_Result.cost > 5) {
        delete point_state_;
        point_state_ = nullptr;
        return false;
    }

    return m_Result.bConverged;
}

double Landmark::EvaluateF(bool bNewZ, double huberThresh) {
    double cost = 0.0;
    // const int nSize = visual_obs.size();
    Matrix23d J23;
    Matrix3d J33;
    Vector3d position;

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
    for (int cam_id = 0; cam_id < 2; cam_id++) {
        CamModel::Ptr camModel = SensorConfig::Instance().GetCamModel(cam_id);
        int i = 0;
        for (auto& visualOb : visual_obs[cam_id]) {
            Vector3f p_cam_f =
                (dRs[cam_id][i] * position + dts[cam_id][i]).cast<float>();

            Matrix23f J23f;
            visualOb->px_reprj = camModel->camToImage(p_cam_f, J23f, cam_id);
            Matrix23d J23d = J23f.cast<double>();

            Vector2d r = (visualOb->px - visualOb->px_reprj).cast<double>();
#if HUBER || 1
            double reprojErr = r.norm();
            double hw = reprojErr > huberThresh ? huberThresh / reprojErr : 1;
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

            Matrix23d J = J23d * dRs[cam_id][i] * J33;

            HTemp.noalias() += J.transpose() * J * hw;
            bTemp.noalias() += J.transpose() * r * hw;

            i++;
        }
    }
    // if(!bNewZ)
    //	H += Matrix3f::Identity()*FLT_EPSILON;
    return cost;
}

bool Landmark::UserDefinedDecentFail() { return zNew[2] < 0; }

void Landmark::AddVisualObservation(VisualObservation::Ptr obs, int cam_id) {
    flag_dead[cam_id] = false;
    flag_dead_all = false;

    num_obs_tracked++;
    valid_obs_num++;

    obs->link_landmark = this;
    obs->link_frame->valid_landmark_num++;
    visual_obs[cam_id].insert(obs);
    last_last_obs_[cam_id] = last_obs_[cam_id];
    last_obs_[cam_id] = obs;

    // Here we compute the ray angle between the former rays and the last ray
    // The ray angle is used to judge if the landmark is a good landmark
    // If the ray angle is too small, it means the landmark is far from the
    // camera and it is not a good landmark maybe it is not a good idea to use
    // the ray angle to judge the landmark?? because the ray angle is not a good
    // metric to describe the landmark
    ray_angle0 = ray_angle;
    Vector3f ray1 = obs->link_frame->state->Rwi * obs->ray_in_cam;

    float minDot = 2;
    for (auto& visualOb : visual_obs[cam_id]) {
        float dot = ((visualOb->link_frame->state->Rwi * visualOb->ray_in_cam)
                         .normalized())
                        .dot(ray1.normalized());
        if (dot < minDot) minDot = dot;
    }
    if (minDot > 0 && minDot < 1) {
        ray_angle = std::max(ray_angle, acosf(minDot));
    }
}

void Landmark::DrawFeatureTrack(cv::Mat& image, cv::Scalar color,
                                int cam_id) const {
    std::set<VisualObservation::Ptr, VisualObservationComparator>
        visual_obs_set(visual_obs[cam_id].begin(), visual_obs[cam_id].end());

    VisualObservation::Ptr front_obs = nullptr;
    VisualObservation::Ptr curr_obs = nullptr;
    if (!flag_dead[0] && !flag_dead[1] && valid_obs_num > 5) {
        color = _PURPLE_SCALAR;
    }
    for (auto& visualOb : visual_obs_set) {
        curr_obs = visualOb;
        if (front_obs == nullptr) {
            front_obs = curr_obs;
            continue;
        }
        if ((curr_obs->px - front_obs->px).squaredNorm() > 900) {
            front_obs = curr_obs;
            continue;
        }
        cv::line(image, cv::Point(front_obs->px.x(), front_obs->px.y()),
                 cv::Point(curr_obs->px.x(), curr_obs->px.y()), _GREEN_SCALAR,
                 1);
        cv::circle(image, cv::Point(curr_obs->px.x(), curr_obs->px.y()), 2,
                   color);
        front_obs = curr_obs;
    }
    cv::circle(image,
               cv::Point(last_obs_[cam_id]->px.x(), last_obs_[cam_id]->px.y()),
               8, color);
}
float Landmark::Reproject(bool verbose, int cam_id) {
    assert(point_state_);
    CamModel::Ptr camModel = SensorConfig::Instance().GetCamModel(0);
    float reprojErr = 0;
    for (auto& ob : visual_obs[cam_id]) {
        ob->px_reprj = camModel->imuToImage(
            ob->link_frame->state->Rwi.transpose() *
            (point_state_->Pw - ob->link_frame->state->Pwi));
        reprojErr += (ob->px_reprj - ob->px).norm();
    }
    reprojErr = visual_obs[cam_id].size() > 0
                    ? reprojErr / visual_obs[cam_id].size()
                    : 100;
    if (reprojErr > 3) {
        if (reprojErr > 5)
            // DrawObservationsAndReprojection();
            if (verbose)
                printf("Reproj Err:%f, Triangulation Cost:%f\n", reprojErr,
                       m_Result.cost);
    }
    return reprojErr;
}

void Landmark::DrawObservationsAndReprojection(int time, int cam_id) {
#if ENABLE_VISUALIZER && !defined(PLATFORM_ARM)
    if (Config::NoGUI) return;
    cv::Mat display;
    bool first = 1;
    for (auto& ob : visual_obs[cam_id]) {
        cv::cvtColor(ob->link_frame->image, display, cv::COLOR_GRAY2BGR);
        if (first) {
            cv::circle(display, cv::Point(ob->px.x(), ob->px.y()), 8,
                       _GREEN_SCALAR);
            cv::circle(display, cv::Point(ob->px_reprj.x(), ob->px_reprj.y()),
                       10, _BLUE_SCALAR);
            first = 0;
        } else {
            cv::circle(display, cv::Point(ob->px.x(), ob->px.y()), 4,
                       _GREEN_SCALAR);
            cv::circle(display, cv::Point(ob->px_reprj.x(), ob->px_reprj.y()),
                       6, _BLUE_SCALAR);
        }
        cv::imshow("ob and reproj", display);
        cv::waitKey(time);
    }
#else
    (void)time;
    (void)cam_id;
#endif
}

void Landmark::PrintObservations(int cam_id) {
    for (auto& visualOb : visual_obs[cam_id]) {
        LOGI("Px: %f %f,Pos:%f %f %f", visualOb->px.x(), visualOb->px.y(),
             visualOb->link_frame->state->Pwi.x(),
             visualOb->link_frame->state->Pwi.y(),
             visualOb->link_frame->state->Pwi.z());
    }
}
void Landmark::RemoveUselessObservationForSlamPoint() {
    // return;
    // I think this function is not needed
    // assert(point_state_ && point_state_->flag_slam_point);
    for (int cam_id = 0; cam_id < 2; cam_id++) {
        for (auto iter = visual_obs[cam_id].begin();
             iter != visual_obs[cam_id].end();) {
            auto visualOb = *iter;
            if (!visualOb->link_frame->flag_keyframe) {
                visualOb->link_landmark = nullptr;
                visualOb->link_frame->valid_landmark_num--;
                iter = visual_obs[cam_id].erase(iter);
                valid_obs_num--;
            } else {
                iter++;
            }
        }
    }
}
bool Landmark::UserDefinedConvergeCriteria() {
    if (m_Result.cost < 10) m_Result.bConverged = true;
    if (m_Result.cost > 40) m_Result.bConverged = false;
    return true;
}

void Landmark::PrintPositions() {}

void Landmark::PopObservation(int cam_id) {
    flag_dead[cam_id] = true;
    visual_obs[cam_id].erase(last_obs_[cam_id]);
    last_obs_[cam_id]->link_landmark = nullptr;
    last_obs_[cam_id] = last_last_obs_[cam_id];
    last_last_obs_[cam_id] = nullptr;
    ray_angle = ray_angle0;
    num_obs_tracked--;
    valid_obs_num--;
}

void Landmark::RemoveLinksInCamStates() {
    for (int cam_id = 0; cam_id < 2; cam_id++) {
        for (auto& ob : visual_obs[cam_id]) {
            ob->link_landmark = nullptr;
            ob->link_frame->valid_landmark_num--;
        }
        visual_obs[cam_id].clear();
    }
    valid_obs_num = 0;
}

}  // namespace DeltaVins
