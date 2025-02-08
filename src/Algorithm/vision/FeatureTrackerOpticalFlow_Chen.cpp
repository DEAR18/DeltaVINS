#include "Algorithm/vision/FeatureTrackerOpticalFlow_Chen.h"

#include <utils/TickTock.h>

#include "Algorithm/DataAssociation/DataAssociation.h"
#include "Algorithm/vision/camModel/camModel.h"
#include "fast/fast.h"
#include "precompile.h"

namespace DeltaVins {
FeatureTrackerOpticalFlow_Chen::FeatureTrackerOpticalFlow_Chen(int nMax2Track,
                                                               int nMaskSize)
    : max_num_to_track_(nMax2Track), mask_size_(nMaskSize) {
    assert(nMaskSize % 2);
}

inline void FeatureTrackerOpticalFlow_Chen::_SetMask(int x, int y) {
    const int imgStride = CamModel::getCamModel()->width();
    const int height = CamModel::getCamModel()->height();
    const int halfMaskSize = (mask_size_ - 1) / 2;

    if (x < halfMaskSize) x = halfMaskSize;
    if (y < halfMaskSize) y = halfMaskSize;
    if (x >= imgStride - halfMaskSize - 1) x = imgStride - halfMaskSize - 2;
    if (y >= height - halfMaskSize - 1) y = height - halfMaskSize - 2;

    unsigned char* pMask0 =
        mask_ + x + (y - halfMaskSize) * imgStride - halfMaskSize;
    unsigned char* pMask1 = pMask0 + imgStride * mask_size_;
    assert(pMask1 + mask_size_ <= mask_ + mask_buffer_size_);
    for (; pMask0 < pMask1; pMask0 += imgStride) {
        memset(pMask0, 0, mask_size_);
    }
}

bool FeatureTrackerOpticalFlow_Chen::_IsMasked(int x, int y) {
    const int imgStride = CamModel::getCamModel()->width();
    return !mask_[x + y * imgStride];
}

void FeatureTrackerOpticalFlow_Chen::_ResetMask() {
    memset(mask_, 0xff, mask_buffer_size_);
}

void FeatureTrackerOpticalFlow_Chen::_ShowMask() {
    int imgStride = CamModel::getCamModel()->width();
    int height = CamModel::getCamModel()->height();
    cv::Mat mask_img(height, imgStride, CV_8UC1, mask_);
    cv::imshow("mask", mask_img);
    cv::waitKey(1);
}

void FeatureTrackerOpticalFlow_Chen::_ExtractMorePoints(
    std::list<TrackedFeaturePtr>& vTrackedFeatures) {
    _ResetMask();
    // Set Mask Pattern
    const int imgStride = CamModel::getCamModel()->width();

    for (auto tf : vTrackedFeatures) {
        if (!tf->flag_dead) {
            auto xy = tf->visual_obs.back().px;
            _SetMask(xy.x(), xy.y());
        }
    }
    // Todo: test mask correctness

    // Extract More Points out of mask
    int halfMaskSize = (mask_size_ - 1) / 2;

    std::vector<cv::Point2f> corners;

#if USE_HARRIS

    _ExtractHarris(corners, max_num_to_track_ - num_features_tracked_);

#else
    _ExtractFast(imgStride, halfMaskSize, corners);

#endif
    std::vector<cv::Point2f> stereo_corners;
    std::vector<cv::Point2f> stereo_corners_back;
    std::vector<unsigned char> stereo_status, stereo_status_back, final_status;
    std::vector<float> err;

    if (CamModel::getCamModel()->IsStereo()) {
        auto cv_point_distance_sqr = [](cv::Point2f a, cv::Point2f b) {
            return ((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
        };
        cv::calcOpticalFlowPyrLK(image_pyramid_, right_image_pyramid_, corners,
                                 stereo_corners, stereo_status, err);
        cv::calcOpticalFlowPyrLK(right_image_pyramid_, image_pyramid_,
                                 stereo_corners, stereo_corners_back,
                                 stereo_status_back, err);
        for (size_t i = 0; i < stereo_corners.size(); i++) {
            if (stereo_status[i] && stereo_status_back[i] &&
                cv_point_distance_sqr(corners[i], stereo_corners_back[i]) < 1) {
                final_status.push_back(1);
            } else {
                final_status.push_back(0);
            }
        }
        // // show stereo match
        // // merge two image
        // cv::Mat img_show;
        // cv::hconcat(image_, right_image_, img_show);
        // cv::cvtColor(img_show, img_show, cv::COLOR_BGR2RGB);
        // for (int i = 0; i < stereo_corners.size(); i++) {
        //     if (final_status[i]) {
        //         cv::circle(img_show, corners[i], 2, cv::Scalar(0, 255, 0),
        //         2); cv::circle(img_show,cv::Point2f(stereo_corners[i].x +
        //         imgStride, stereo_corners[i].y), 2, cv::Scalar(0, 255, 0),
        //         2);
        //         cv::line(img_show,corners[i],cv::Point2f(stereo_corners[i].x
        //         + imgStride, stereo_corners[i].y),cv::Scalar(255, 0, 0),1);
        //     }
        // }
        // cv::imshow("stereo match",img_show);
        // cv::waitKey(0);
    }

    for (size_t i = 0, j = max_num_to_track_ - num_features_tracked_;
         i < corners.size() && j > 0; ++i) {
        int x = corners[i].x;
        int y = corners[i].y;
        assert(x + y * imgStride < mask_buffer_size_);
        if (mask_[x + y * imgStride]) {
            _SetMask(x, y);
            j--;
            auto tf = std::make_shared<TrackedFeature>();
            if (CamModel::getCamModel()->IsStereo()) {
                // auto left_model = CamModel::getCamModel(0);
                // auto right_model = CamModel::getCamModel(1);
                if (final_status[i]) {
                    // Vector2f px1 =
                    //     left_model->camToImage(left_model->imageToCam(
                    //         Vector2f(corners[i].x, corners[i].y)));
                    // Vector2f px2 = right_model->camToImage(
                    //     right_model->imageToCam(Vector2f(stereo_corners[i].x,
                    //                                      stereo_corners[i].y)));
                    // if (!left_model->inView(px1)) continue;
                    // if (!right_model->inView(px2)) continue;
                    // float depth_prior =
                    //     CamModel::getCamModel()->depthFromStereo(
                    //         px1,
                    //         px2);
                    // if(depth_prior<0){
                    //     LOGE("invalid depth_error:%f",depth_prior);
                    // }
                    // tf->AddVisualObservation(Vector2f(x, y), cam_state_,
                    //                          depth_prior);
                } else {
                    tf->AddVisualObservation(Vector2f(x, y), cam_state_);
                }
            } else {
                tf->AddVisualObservation(Vector2f(x, y), cam_state_);
            }
            vTrackedFeatures.push_back(tf);
            ++num_features_;
        }
    }
    if (vTrackedFeatures.empty()) {
        LOGW("No more features detected");
    }
}

void FeatureTrackerOpticalFlow_Chen::_TrackPoints(
    std::list<TrackedFeaturePtr>& vTrackedFeatures) {
    if (last_image_.empty()) return;
    Matrix3f dR = cam_state_->state->Rwi.transpose() * cam_state0_->state->Rwi;
    std::vector<cv::Point2f> pre, now;
    std::vector<unsigned char> status;
    std::vector<TrackedFeature*> goodTracks;
    goodTracks.reserve(vTrackedFeatures.size());
    pre.reserve(vTrackedFeatures.size());
    now.reserve(vTrackedFeatures.size());

    auto camModel = CamModel::getCamModel();
    // float mean_moved_pixels = 0;
    for (auto tf : vTrackedFeatures) {
        if (!tf->flag_dead) {
            auto& lastVisualOb = tf->visual_obs.back();

#if USE_ROTATION_PREDICTION
            Vector3f ray = dR * lastVisualOb.ray;

            Vector2f px = camModel->camToImage(ray);
            if (!camModel->inView(px)) {
                tf->flag_dead = true;
                continue;
            }
            now.emplace_back(px.x(), px.y());
            pre.emplace_back(lastVisualOb.px.x(), lastVisualOb.px.y());
            goodTracks.push_back(tf.get());
            tf->predicted_px = px;
#else
#if 0
                Vector3f ray = dR * lastVisualOb.ray;

                static Matrix3f Rci = camModel->getRci();
                Vector2f px = camModel->camToImage(Rci * ray);
                tf->predicted_px = px;
#endif
            pre.emplace_back(lastVisualOb.px.x(), lastVisualOb.px.y());
            now.emplace_back(pre.back());
            goodTracks.push_back(tf.get());
#endif
        }
    }
    // int nPrePoints = pre.size();
    if (pre.empty()) {
        LOGW("No feature to track.");
        return;
    }
    bool use_predict = false;
#if USE_ROTATION_PREDICTION
    use_predict = true;
#endif
    std::vector<float> err;
    if (use_cache_) {
        cv::calcOpticalFlowPyrLK(
            last_image_pyramid_, image_pyramid_, pre, now, status, err,
            cv::Size(21, 21), 3,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                             30, 0.01),
            use_predict ? cv::OPTFLOW_USE_INITIAL_FLOW |
                              cv::OPTFLOW_LK_GET_MIN_EIGENVALS
                        : 0,
            5e-3);
    } else {
        cv::calcOpticalFlowPyrLK(
            last_image_, image_, pre, now, status, err, cv::Size(21, 21), 3,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                             30, 0.01),
            use_predict ? cv::OPTFLOW_USE_INITIAL_FLOW |
                              cv::OPTFLOW_LK_GET_MIN_EIGENVALS
                        : 0,
            5e-3);
    }
    bool do_back_track = true;
    // back track
    std::vector<cv::Point2f> back_track_pre;
    std::vector<unsigned char> back_track_status;
    if (do_back_track) {
        if (use_cache_) {
            cv::calcOpticalFlowPyrLK(image_pyramid_, last_image_pyramid_, now,
                                     back_track_pre, back_track_status, err,
                                     cv::Size(21, 21), 3);
        } else {
            cv::calcOpticalFlowPyrLK(image_, last_image_, now, back_track_pre,
                                     back_track_status, err, cv::Size(21, 21),
                                     3);
        }
    }

    for (size_t i = 0; i < status.size(); ++i) {
        Vector2f px;
        if (status[i]) {
            if (do_back_track) {
                if (!back_track_status[i] ||
                    cv::normL2Sqr(&pre[i].x, &back_track_pre[i].x, 2) > 1) {
                    goodTracks[i]->flag_dead = true;
                    continue;
                }
            }
            px.x() = now[i].x;
            px.y() = now[i].y;
            if (camModel->inView(px, 5)) {
                // if (goodTracks[i]->point_state_ || !_IsMasked(px.x(),
                // px.y())) {
                num_features_++;
                num_features_tracked_++;
                goodTracks[i]->AddVisualObservation(px, cam_state_);
                // _SetMask(px.x(), px.y());
                // }
                continue;
            }
        }
        goodTracks[i]->flag_dead = true;
    }

    int nRansac =
        DataAssociation::RemoveOutlierBy2PointRansac(dR, vTrackedFeatures);

    // LOGW("nPointsLast:%d nPointsTracked:%d nPointsAfterRansac:%d",
    // pre.size(),
    //      num_features_tracked_, nRansac);
    num_features_tracked_ = nRansac;
}

void FeatureTrackerOpticalFlow_Chen::MatchNewFrame(
    std::list<TrackedFeaturePtr>& vTrackedFeatures, const ImageData::Ptr image,
    Frame* camState) {
    num_features_ = 0;
    image_ = image->image;
    if (CamModel::getCamModel()->IsStereo()) {
        right_image_ = image->right_image;
    }
    if (use_cache_) {
        image_pyramid_.clear();
        cv::buildOpticalFlowPyramid(image_, image_pyramid_, cv::Size(21, 21),
                                    3);
        if (CamModel::getCamModel()->IsStereo()) {
            right_image_pyramid_.clear();
            cv::buildOpticalFlowPyramid(right_image_, right_image_pyramid_,
                                        cv::Size(21, 21), 3);
        }
    }

    cam_state0_ = cam_state_;
    cam_state_ = camState;

    // Set cnt for tracked points to zero
    num_features_tracked_ = 0;

    // Init mask buffer
    if (mask_ == nullptr) {
        mask_buffer_size_ = CamModel::getCamModel()->area();
        mask_ = new unsigned char[mask_buffer_size_];
    }
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
    last_image_ = image_;
    if (use_cache_) {
        last_image_pyramid_ = image_pyramid_;
    }
}

FeatureTrackerOpticalFlow_Chen::~FeatureTrackerOpticalFlow_Chen() {
    delete[] mask_;
    mask_ = nullptr;
}

void FeatureTrackerOpticalFlow_Chen::_ExtractFast(
    const int imgStride, const int halfMaskSize,
    std::vector<cv::Point2f>& corner) {
    std::vector<fast::fast_xy> vXys;
    std::vector<int> vScores, vNms;
    std::vector<std::pair<int, fast::fast_xy>> vTemp;
    fast::fast_corner_detect_10_mask(
        image_.data + halfMaskSize + halfMaskSize * imgStride,
        mask_ + halfMaskSize + halfMaskSize * imgStride,
        image_.cols - mask_size_ + 1, image_.rows - mask_size_ + 1,
        image_.step1(), 15, vXys);
    fast::fast_corner_score_10(
        image_.data + halfMaskSize + halfMaskSize * imgStride, image_.step1(),
        vXys, 15, vScores);
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
    std::vector<cv::Point2f>& corners, int max_num) {
    auto camModel = CamModel::getCamModel();
    cv::Mat mask(camModel->height(), camModel->width(), CV_8UC1, mask_);
    cv::goodFeaturesToTrack(image_, corners, max_num, 0.1, 20, mask);
}
}  // namespace DeltaVins
