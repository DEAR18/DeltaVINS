#include "Algorithm/vision/FeatureTrackerOpticalFlow_Chen.h"

#include <utils/TickTock.h>

#include "Algorithm/DataAssociation/DataAssociation.h"
#include "Algorithm/vision/camModel/camModel.h"
#include "fast/fast.h"
#include "precompile.h"
#include "utils/SensorConfig.h"
namespace DeltaVins {
FeatureTrackerOpticalFlow_Chen::FeatureTrackerOpticalFlow_Chen(int nMax2Track,
                                                               int nMaskSize)
    : max_num_to_track_(nMax2Track), mask_size_(nMaskSize) {
    assert(nMaskSize % 2);
    use_back_tracking_ = Config::UseBackTracking;
}

inline void FeatureTrackerOpticalFlow_Chen::_SetMask(int x, int y, int cam_id) {
    auto camModel = SensorConfig::Instance().GetCamModel(image_->sensor_id);
    const int imgStride = camModel->width();
    const int height = camModel->height();
    const int halfMaskSize = (mask_size_ - 1) / 2;

    if (x < halfMaskSize) x = halfMaskSize;
    if (y < halfMaskSize) y = halfMaskSize;
    if (x >= imgStride - halfMaskSize - 1) x = imgStride - halfMaskSize - 2;
    if (y >= height - halfMaskSize - 1) y = height - halfMaskSize - 2;

    unsigned char* pMask0 = nullptr;
    unsigned char* pMask1 = nullptr;
    if (cam_id == 0) {
        pMask0 = mask_ + x + (y - halfMaskSize) * imgStride - halfMaskSize;
        pMask1 = pMask0 + imgStride * mask_size_;
        assert(pMask1 + mask_size_ <= mask_ + mask_buffer_size_);
    } else {
        pMask0 =
            mask_right_ + x + (y - halfMaskSize) * imgStride - halfMaskSize;
        pMask1 = pMask0 + imgStride * mask_size_;
        assert(pMask1 + mask_size_ <= mask_right_ + mask_buffer_size_);
    }
    for (; pMask0 < pMask1; pMask0 += imgStride) {
        memset(pMask0, 0, mask_size_);
    }
}

bool FeatureTrackerOpticalFlow_Chen::IsStaticLastFrame() {
    int nPxStatic = 0;
    int nAllPx = 0;
    float ratioThresh = 0.3;

    float pxThreshSqr = 0.5 * 0.5;

    for (auto moved_px : last_frame_moved_pixels_sqr_) {
        if (moved_px < pxThreshSqr) nPxStatic++;
        nAllPx++;
    }

    if (nAllPx == 0) return false;

    if (float(nPxStatic) / float(nAllPx) > ratioThresh) return true;
    return false;
}

bool FeatureTrackerOpticalFlow_Chen::_IsMasked(int x, int y, int cam_id) {
    auto camModel = SensorConfig::Instance().GetCamModel(image_->sensor_id);
    const int imgStride = camModel->width();
    if (cam_id == 0) {
        return !mask_[x + y * imgStride];
    } else {
        return !mask_right_[x + y * imgStride];
    }
}

void FeatureTrackerOpticalFlow_Chen::_ResetMask() {
    memset(mask_, 0xff, mask_buffer_size_);
    if (SensorConfig::Instance().GetCamModel(image_->sensor_id)->IsStereo()) {
        memset(mask_right_, 0xff, mask_buffer_size_);
    }
}

void FeatureTrackerOpticalFlow_Chen::_ShowMask() {
    auto camModel = SensorConfig::Instance().GetCamModel(image_->sensor_id);
    int imgStride = camModel->width();
    int height = camModel->height();
    cv::Mat mask_img(height, imgStride, CV_8UC1, mask_);
    cv::imshow("mask", mask_img);
    cv::waitKey(1);
}

void FeatureTrackerOpticalFlow_Chen::_ExtractMorePoints(
    std::list<LandmarkPtr>& vTrackedFeatures) {
    _ResetMask();
    // Set Mask Pattern
    auto camModel = SensorConfig::Instance().GetCamModel(image_->sensor_id);
    const int imgStride = camModel->width();

    bool is_stereo = camModel->IsStereo();
    for (auto tf : vTrackedFeatures) {
        if (!tf->flag_dead_all) {
            for (int cam_id = 0; cam_id < 2; cam_id++) {
                if (tf->flag_dead[cam_id]) {
                    continue;
                }
                if (tf->last_obs_[cam_id]) {
                    auto xy = tf->last_obs_[cam_id]->px;
                    _SetMask(xy.x(), xy.y(), cam_id);
                }
            }
        }
    }
    // Todo: test mask correctness

    // Extract More Points out of mask
    int halfMaskSize = (mask_size_ - 1) / 2;

    std::vector<cv::Point2f> corners;
    std::vector<cv::Point2f> corners_right;

    // Step 1: we extract left image features

#if USE_HARRIS
    _ExtractHarris(corners, max_num, 0);
#else
    _ExtractFast(imgStride, halfMaskSize, corners, 0);
#endif

    std::vector<cv::Point2f> stereo_corners;
    std::vector<cv::Point2f> stereo_corners_back;
    std::vector<unsigned char> stereo_status, stereo_status_back, final_status;
    std::vector<float> err;

    if (!corners.empty()) {
        // Step 2: we find the right stereo features if stereo is enabled
        if (is_stereo) {
            cv::calcOpticalFlowPyrLK(image_pyramid_[0], image_pyramid_[1],
                                     corners, stereo_corners, stereo_status,
                                     err);
            cv::calcOpticalFlowPyrLK(image_pyramid_[1], image_pyramid_[0],
                                     stereo_corners, stereo_corners_back,
                                     stereo_status_back, err);
            for (size_t i = 0; i < stereo_corners.size(); i++) {
                if (stereo_status[i] && stereo_status_back[i] &&
                    cv::normL2Sqr(&corners[i].x, &stereo_corners_back[i].x, 2) <
                        1) {
                    final_status.push_back(1);
                } else {
                    final_status.push_back(0);
                }
            }
        }

        // Step 3: we add left only features and left to right stereo features
        for (size_t corner_idx = 0, track_candidates = max_num_to_track_ -
                                                       num_features_tracked_;
             corner_idx < corners.size() && track_candidates > 0;
             ++corner_idx) {
            int x = corners[corner_idx].x;
            int y = corners[corner_idx].y;
            assert(x + y * imgStride < mask_buffer_size_);
            if (!_IsMasked(x, y, 0)) {
                _SetMask(x, y, 0);
                track_candidates--;
                auto tf = std::make_shared<Landmark>();
                auto obs = cam_state_->AddVisualObservation(Vector2f(x, y), 0);
                if (is_stereo) {
                    if (final_status[corner_idx]) {
                        // Add stereo observation
                        auto obs_right = cam_state_->AddVisualObservation(
                            Vector2f(stereo_corners[corner_idx].x,
                                     stereo_corners[corner_idx].y),
                            1);
                        _SetMask(stereo_corners[corner_idx].x,
                                 stereo_corners[corner_idx].y, 1);
                        obs_right->stereo_obs = obs.get();
                        obs->stereo_obs = obs_right.get();
                        tf->AddVisualObservation(obs_right, 1);
                        tf->AddVisualObservation(obs, 0);
                    } else {
                        tf->AddVisualObservation(obs, 0);
                    }
                } else {
                    tf->AddVisualObservation(obs, 0);
                }
                vTrackedFeatures.push_back(tf);
                ++num_features_;
                ++num_features_tracked_;
            }
        }
    }

    // Step 4: we add right only features and right to left stereo features
    if (is_stereo) {
        corners_right.clear();
        stereo_corners.clear();
        stereo_corners_back.clear();
        stereo_status.clear();
        stereo_status_back.clear();
#if USE_HARRIS
        _ExtractHarris(corners_right, max_num, 1);
#else
        _ExtractFast(imgStride, halfMaskSize, corners_right, 1);
#endif
        if (!corners_right.empty()) {
            // find right to left stereo features
            cv::calcOpticalFlowPyrLK(image_pyramid_[1], image_pyramid_[0],
                                     corners_right, stereo_corners,
                                     stereo_status, err);
            cv::calcOpticalFlowPyrLK(image_pyramid_[0], image_pyramid_[1],
                                     stereo_corners, stereo_corners_back,
                                     stereo_status_back, err);
            for (size_t i = 0; i < stereo_corners.size(); i++) {
                if (stereo_status[i] && stereo_status_back[i] &&
                    cv::normL2Sqr(&corners_right[i].x,
                                  &stereo_corners_back[i].x, 2) < 1) {
                    final_status.push_back(1);
                } else {
                    final_status.push_back(0);
                }
            }
            for (size_t corner_idx = 0,
                        track_candidates =
                            max_num_to_track_ - num_features_tracked_;
                 corner_idx < corners_right.size() && track_candidates > 0;
                 ++corner_idx) {
                int x = corners_right[corner_idx].x;
                int y = corners_right[corner_idx].y;
                assert(x + y * imgStride < mask_buffer_size_);
                if (!_IsMasked(x, y, 1)) {
                    _SetMask(x, y, 1);
                    track_candidates--;
                    auto tf = std::make_shared<Landmark>();
                    auto obs =
                        cam_state_->AddVisualObservation(Vector2f(x, y), 1);
                    if (final_status[corner_idx]) {
                        // Add stereo observation
                        auto obs_left = cam_state_->AddVisualObservation(
                            Vector2f(stereo_corners[corner_idx].x,
                                     stereo_corners[corner_idx].y),
                            0);
                        // no need to set mask here, because we won't use mask
                        // after this _SetMask(stereo_corners[corner_idx].x,
                        //  stereo_corners[corner_idx].y, 0);
                        obs_left->stereo_obs = obs.get();
                        obs->stereo_obs = obs_left.get();
                        tf->AddVisualObservation(obs_left, 0);
                    } else {
                        tf->AddVisualObservation(obs, 1);
                    }
                    vTrackedFeatures.push_back(tf);
                    ++num_features_;
                    ++num_features_tracked_;
                }
            }
        }
    }

    if (vTrackedFeatures.empty()) {
        LOGW("No more features detected");
    }
}

void FeatureTrackerOpticalFlow_Chen::_TrackFromLastFrame(
    std::list<LandmarkPtr>& vTrackedFeatures, int cam_id) {
    auto camModel = SensorConfig::Instance().GetCamModel(image_->sensor_id);

    Matrix3f Rci = camModel->getRci(cam_id);
    Matrix3f dR = Rci * cam_state_->state->Rwi.transpose() *
                  cam_state0_->state->Rwi * Rci.transpose();

    std::vector<cv::Point2f> pre, now;
    std::vector<unsigned char> status;
    std::vector<Landmark*> goodTracks;
    goodTracks.reserve(vTrackedFeatures.size());
    pre.reserve(vTrackedFeatures.size());
    now.reserve(vTrackedFeatures.size());

    for (auto tf : vTrackedFeatures) {
        if (!tf->flag_dead[cam_id]) {
            auto& lastVisualOb = tf->last_obs_[cam_id];
            if (!lastVisualOb) continue;

#if USE_ROTATION_PREDICTION
            Vector3f ray = dR * lastVisualOb->ray_in_cam;

            Vector2f px = camModel->camToImage(ray);
            if (!camModel->inView(px, cam_id)) {
                tf->flag_dead[cam_id] = true;
                continue;
            }
            now.emplace_back(px.x(), px.y());
            pre.emplace_back(lastVisualOb->px.x(), lastVisualOb->px.y());
            goodTracks.push_back(tf.get());
            tf->predicted_px[cam_id] = px;
#else
            pre.emplace_back(lastVisualOb->px.x(), lastVisualOb->px.y());
            now.emplace_back(pre.back());
            goodTracks.push_back(tf.get());
#endif
        }
    }
    if (pre.empty()) {
        LOGW("No feature to track.");
        return;
    }
    bool use_predict = false;
#if USE_ROTATION_PREDICTION
    use_predict = true;
#endif
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(
        last_image_pyramid_[cam_id], image_pyramid_[cam_id], pre, now, status,
        err, cv::Size(21, 21), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
        use_predict
            ? cv::OPTFLOW_USE_INITIAL_FLOW | cv::OPTFLOW_LK_GET_MIN_EIGENVALS
            : 0,
        5e-3);
    // back track
    std::vector<cv::Point2f> back_track_pre;
    std::vector<unsigned char> back_track_status;
    if (use_back_tracking_) {
        cv::calcOpticalFlowPyrLK(
            image_pyramid_[cam_id], last_image_pyramid_[cam_id], now,
            back_track_pre, back_track_status, err, cv::Size(21, 21), 3);
    }

    for (size_t i = 0; i < status.size(); ++i) {
        Vector2f px;
        if (status[i]) {
            if (use_back_tracking_) {
                if (!back_track_status[i] ||
                    cv::normL2Sqr(&pre[i].x, &back_track_pre[i].x, 2) > 1) {
                    goodTracks[i]->flag_dead[cam_id] = true;
                    continue;
                }
            }
            px.x() = now[i].x;
            px.y() = now[i].y;
            if (camModel->inView(px, cam_id)) {
                auto obs = cam_state_->AddVisualObservation(px, cam_id);
                num_features_++;
                num_features_tracked_++;
                goodTracks[i]->AddVisualObservation(obs, cam_id);
                last_frame_moved_pixels_sqr_.push_back(
                    cv::normL2Sqr(&now[i].x, &pre[i].x, 2));
                continue;
            }
        }
        goodTracks[i]->flag_dead[cam_id] = true;
    }
}

void FeatureTrackerOpticalFlow_Chen::_TrackStereoFeatures(
    std::list<LandmarkPtr>& vTrackedFeatures) {
    // track left to right and right to left stereo features
    // Step 1: we find all the stereo features
    std::vector<LandmarkPtr> stereo_features;
    std::vector<cv::Point2f> left, right;
    std::vector<cv::Point2f> left2right, right2left;
    std::vector<unsigned char> left2right_status, right2left_status;
    std::vector<float> left2right_err, right2left_err;
    for (auto tf : vTrackedFeatures) {
        if (tf->flag_dead[0] || tf->flag_dead[1]) continue;
        stereo_features.push_back(tf);
        left.push_back(
            cv::Point2f(tf->last_obs_[0]->px.x(), tf->last_obs_[0]->px.y()));
        right.push_back(
            cv::Point2f(tf->last_obs_[1]->px.x(), tf->last_obs_[1]->px.y()));
    }

    // Step 2: we track left to right and right to left stereo features
    if (stereo_features.empty()) {
        return;
    }
    cv::calcOpticalFlowPyrLK(image_pyramid_[0], image_pyramid_[1], left,
                             left2right, left2right_status, left2right_err);
    cv::calcOpticalFlowPyrLK(image_pyramid_[1], image_pyramid_[0], right,
                             right2left, right2left_status, right2left_err);

    // Step 3: we add left to right and right to left stereo features
    for (size_t i = 0; i < left.size(); i++) {
        if (left2right_status[i] && right2left_status[i] &&
            cv::normL2Sqr(&left[i].x, &right2left[i].x, 2) < 1 &&
            cv::normL2Sqr(&right[i].x, &left2right[i].x, 2) < 1) {
            // add stereo observation
            auto tf = stereo_features[i];
            tf->last_obs_[0]->stereo_obs = tf->last_obs_[1].get();
            tf->last_obs_[1]->stereo_obs = tf->last_obs_[0].get();
        } else {
            auto tf = stereo_features[i];
            tf->flag_dead[0] = true;
            tf->flag_dead[1] = true;
            tf->PopObservation(0);
            tf->PopObservation(1);
        }
    }
}

void FeatureTrackerOpticalFlow_Chen::_TrackPoints(
    std::list<LandmarkPtr>& vTrackedFeatures) {
    if (last_image_ == nullptr) return;
    if (vTrackedFeatures.empty()) return;
    // Step 1: we track left image features and right image features if stereo
    // is enabled
    bool is_stereo =
        SensorConfig::Instance().GetCamModel(image_->sensor_id)->IsStereo();
    int cam_num = is_stereo ? 2 : 1;
    for (int cam_id = 0; cam_id < cam_num; cam_id++) {
        _TrackFromLastFrame(vTrackedFeatures, cam_id);
    }

    // Step 2: we track left to right and right to left stereo features
    if (is_stereo) {
        _TrackStereoFeatures(vTrackedFeatures);
    }

    // int nRansac = DataAssociation::RemoveOutlierBy2PointRansac(
    //     dR, vTrackedFeatures, image_->sensor_id);

    // num_features_tracked_ = nRansac;

    // LOGW("nPointsLast:%zu nPointsTracked:%d",
    //      pre.size(), num_features_tracked_);
    // static FILE* fp = fopen("track_points.txt", "w");
    // fprintf(fp, "%f\n", num_features_tracked_ / float(pre.size()));
}

void FeatureTrackerOpticalFlow_Chen::_PreProcess(const ImageData::Ptr image,
                                                 Frame* camState) {
    num_features_ = 0;
    image_ = image;
    image_pyramid_[0].clear();
    cv::buildOpticalFlowPyramid(image_->image, image_pyramid_[0],
                                cv::Size(21, 21), 3);
    if (SensorConfig::Instance().GetCamModel(image_->sensor_id)->IsStereo()) {
        image_pyramid_[1].clear();
        cv::buildOpticalFlowPyramid(image_->right_image, image_pyramid_[1],
                                    cv::Size(21, 21), 3);
    }
    cam_state0_ = cam_state_;
    cam_state_ = camState;

    // Set cnt for tracked points to zero
    num_features_tracked_ = 0;
    last_frame_moved_pixels_sqr_.clear();

    // Init mask buffer
    if (mask_ == nullptr) {
        auto camModel = SensorConfig::Instance().GetCamModel(image_->sensor_id);
        mask_buffer_size_ = camModel->area();
        mask_ = new unsigned char[mask_buffer_size_];
    }
    if (SensorConfig::Instance().GetCamModel(image_->sensor_id)->IsStereo()) {
        if (mask_right_ == nullptr) {
            auto camModel =
                SensorConfig::Instance().GetCamModel(image_->sensor_id);
            mask_buffer_size_ = camModel->area(1);
            mask_right_ = new unsigned char[mask_buffer_size_];
        }
    }
}

void FeatureTrackerOpticalFlow_Chen::_PostProcess(
    std::list<LandmarkPtr>& vTrackedFeatures) {
    last_image_ = image_;
    last_image_pyramid_[0] = image_pyramid_[0];
    if (SensorConfig::Instance().GetCamModel(image_->sensor_id)->IsStereo()) {
        last_image_pyramid_[1] = image_pyramid_[1];
    }
    for (auto tf : vTrackedFeatures) {
        if (tf->flag_dead[0] && tf->flag_dead[1]) {
            tf->flag_dead_all = true;
        }
    }
}

void FeatureTrackerOpticalFlow_Chen::MatchNewFrame(
    std::list<LandmarkPtr>& vTrackedFeatures, const ImageData::Ptr image,
    Frame* camState) {
    _PreProcess(image, camState);

    // ReSet Mask Pattern
    // _ResetMask();

    TickTock::Start("KLT");
    _TrackPoints(vTrackedFeatures);
    TickTock::Stop("KLT");

    // Extract more points if there are more points can be tracked.
    if (num_features_tracked_ < max_num_to_track_) {
        TickTock::Start("Fast");
        _ExtractMorePoints(vTrackedFeatures);
        TickTock::Stop("Fast");
    }
    // _ShowMask();
    _PostProcess(vTrackedFeatures);
}

FeatureTrackerOpticalFlow_Chen::~FeatureTrackerOpticalFlow_Chen() {
    delete[] mask_;
    mask_ = nullptr;
}

void FeatureTrackerOpticalFlow_Chen::_ExtractFast(
    const int imgStride, const int halfMaskSize,
    std::vector<cv::Point2f>& corner, int cam_id) {
    std::vector<fast::fast_xy> vXys;
    std::vector<int> vScores, vNms;
    std::vector<std::pair<int, fast::fast_xy>> vTemp;
    unsigned char* image_data = nullptr;
    unsigned char* mask_data = nullptr;
    if (cam_id == 0) {
        image_data =
            image_->image.data + halfMaskSize + halfMaskSize * imgStride;
        mask_data = mask_ + halfMaskSize + halfMaskSize * imgStride;
    } else {
        image_data =
            image_->right_image.data + halfMaskSize + halfMaskSize * imgStride;
        mask_data = mask_right_ + halfMaskSize + halfMaskSize * imgStride;
    }
    fast::fast_corner_detect_10_mask(
        image_data, mask_data, image_->image.cols - mask_size_ + 1,
        image_->image.rows - mask_size_ + 1, image_->image.step1(),
        Config::FastScoreThreshold, vXys);
    fast::fast_corner_score_10(image_data, image_->image.step1(), vXys,
                               Config::FastScoreThreshold, vScores);
    fast::fast_nonmax_3x3(vXys, vScores, vNms);

    vTemp.reserve(vXys.size());
    for (auto& idx : vNms) {
        vTemp.emplace_back(vScores[idx], vXys[idx]);
    }

#if USE_STABLE_SORT
    std::stable_sort(vTemp.begin(), vTemp.end(),
                     [](const std::pair<int, fast::fast_xy>& a,
                        const std::pair<int, fast::fast_xy>& b) {
                         return a.first > b.first;
                     });
#else

    std::sort(vTemp.begin(), vTemp.end(),
              [](auto& a, auto& b) { return a.first > b.first; });
#endif

    corner.reserve(vTemp.size());
    for (auto& v : vTemp) {
        corner.emplace_back(v.second.x + halfMaskSize,
                            v.second.y + halfMaskSize);
    }
    if (vTemp.empty()) {
        LOGW("No more features detected");
    }
}

void FeatureTrackerOpticalFlow_Chen::_ExtractHarris(
    std::vector<cv::Point2f>& corners, int max_num, int cam_id) {
    auto camModel = SensorConfig::Instance().GetCamModel(image_->sensor_id);
    unsigned char* mask_data = nullptr;
    mask_data = cam_id == 0 ? mask_ : mask_right_;
    cv::Mat mask(camModel->height(), camModel->width(), CV_8UC1, mask_data);
    cv::Mat image = cam_id == 0 ? image_->image : image_->right_image;
    cv::goodFeaturesToTrack(image_->image, corners, max_num, 0.1, 20, mask);
}
}  // namespace DeltaVins
