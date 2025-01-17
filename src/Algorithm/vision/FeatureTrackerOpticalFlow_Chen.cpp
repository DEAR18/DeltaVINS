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

void FeatureTrackerOpticalFlow_Chen::_ExtractMorePoints(
    std::list<TrackedFeaturePtr>& vTrackedFeatures) {
    // ReSet Mask Pattern
    memset(mask_, 1, mask_buffer_size_);

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

    for (int i = 0, j = max_num_to_track_ - num_features_tracked_; i < corners.size() && j > 0;
         ++i) {
        int x = corners[i].x;
        int y = corners[i].y;
        assert(x + y * imgStride < mask_buffer_size_);
        if (mask_[x + y * imgStride]) {
            _SetMask(x, y);
            j--;
            auto tf = std::make_shared<TrackedFeature>();
            tf->AddVisualObservation(Vector2f(x, y), cam_state_);
            vTrackedFeatures.push_back(tf);
            ++num_features_;
        }
    }
}

void FeatureTrackerOpticalFlow_Chen::_TrackPoints(
    std::list<TrackedFeaturePtr>& vTrackedFeatures) {
    if (last_image_.empty()) return;
    Matrix3f dR =
        cam_state_->state->Rwi.transpose() * cam_state0_->state->Rwi;

#if CV_MAJOR_VERSION == 3
    //        auto lkOpticalFlow =
    //        cv::SparsePyrLKOpticalFlow::create(cv::Size(11,11),2,cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS,15,0.01));
    auto lkOpticalFlow = cv::SparsePyrLKOpticalFlow::create();
#endif
    std::vector<cv::Point2f> pre, now;
    std::vector<unsigned char> status;
    std::vector<TrackedFeature*> goodTracks;
    goodTracks.reserve(vTrackedFeatures.size());
    pre.reserve(vTrackedFeatures.size());
    now.reserve(vTrackedFeatures.size());

    auto camModel = CamModel::getCamModel();
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
    int nPrePoints = pre.size();
    if (pre.empty()) {
        LOGW("No feature to track.")
        return;
    }
    bool use_predict = false;
#if USE_ROTATION_PREDICTION
    use_predict = true;
#endif
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(
        last_image_, image_, pre, now, status, err, cv::Size(21, 21), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
        use_predict
            ? cv::OPTFLOW_USE_INITIAL_FLOW | cv::OPTFLOW_LK_GET_MIN_EIGENVALS
            : 0,
        5e-3);

    for (int i = 0; i < status.size(); ++i) {
        Vector2f px;
        if (status[i]) {
            px.x() = now[i].x;
            px.y() = now[i].y;
            if (camModel->inView(px, 5)) {
                num_features_++;
                num_features_tracked_++;
                goodTracks[i]->AddVisualObservation(px, cam_state_);
                continue;
            }
        }

        goodTracks[i]->flag_dead = true;
    }

    int nRansac =
        DataAssociation::RemoveOutlierBy2PointRansac(dR, vTrackedFeatures);

    // LOGW("nPointsLast:%d nPointsTracked:%d nPointsAfterRansac:%d",
    // pre.size(), num_features_tracked_, nRansac);
}

void FeatureTrackerOpticalFlow_Chen::MatchNewFrame(
    std::list<TrackedFeaturePtr>& vTrackedFeatures, cv::Mat& image,
    Frame* camState) {
    num_features_ = 0;
    image_ = image;
    cam_state0_ = cam_state_;
    cam_state_ = camState;

    // Set cnt for tracked points to zero
    num_features_tracked_ = 0;

    // Init mask buffer
    if (mask_ == nullptr) {
        mask_buffer_size_ = CamModel::getCamModel()->area();
        mask_ = new unsigned char[mask_buffer_size_];
    }
    TickTock::Start("KLT");
    _TrackPoints(vTrackedFeatures);
    TickTock::Stop("KLT");
    // Extract more points if there are more points can be tracked.
    if (num_features_tracked_ < max_num_to_track_) {
        TickTock::Start("Fast");
        _ExtractMorePoints(vTrackedFeatures);
        TickTock::Stop("Fast");
    }

    last_image_ = image_;
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
    ;
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
}

void FeatureTrackerOpticalFlow_Chen::_ExtractHarris(
    std::vector<cv::Point2f>& corners, int max_num) {
    cv::Mat mask(480, 640, CV_8UC1, mask_);
    cv::goodFeaturesToTrack(image_, corners, max_num, 0.1, 20, mask);
}
}  // namespace DeltaVins
