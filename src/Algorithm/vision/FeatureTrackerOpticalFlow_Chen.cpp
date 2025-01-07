#include "precompile.h"
#include "fast/fast.h"
#include "Algorithm/vision/FeatureTrackerOpticalFlow_Chen.h"

#include "Algorithm/DataAssociation/DataAssociation.h"
#include "Algorithm/vision/camModel/camModel.h"
#include <utils/TickTock.h>

namespace DeltaVins {
FeatureTrackerOpticalFlow_Chen::FeatureTrackerOpticalFlow_Chen(int nMax2Track, int nMaskSize)
    : m_nMax2Track(nMax2Track), m_nMaskSize(nMaskSize) {
    assert(nMaskSize % 2);
}

inline void FeatureTrackerOpticalFlow_Chen::_setMask(int x, int y) {
    const int imgStride    = CamModel::getCamModel()->width();
    const int height       = CamModel::getCamModel()->height();
    const int halfMaskSize = (m_nMaskSize - 1) / 2;

    if (x < halfMaskSize) x = halfMaskSize;
    if (y < halfMaskSize) y = halfMaskSize;
    if (x >= imgStride - halfMaskSize - 1) x = imgStride - halfMaskSize - 2;
    if (y >= height - halfMaskSize - 1) y = height - halfMaskSize - 2;

    unsigned char* pMask0 = m_pMask + x + (y - halfMaskSize) * imgStride - halfMaskSize;
    unsigned char* pMask1 = pMask0 + imgStride * m_nMaskSize;
    assert(pMask1 + m_nMaskSize <= m_pMask + m_nMaskBufferSize);
    for (; pMask0 < pMask1; pMask0 += imgStride) {
        memset(pMask0, 0, m_nMaskSize);
    }
}

void FeatureTrackerOpticalFlow_Chen::_extractMorePoints(std::list<TrackedFeaturePtr>& vTrackedFeatures) {
    //ReSet Mask Pattern
    memset(m_pMask, 1, m_nMaskBufferSize);

    //Set Mask Pattern
    const int imgStride = CamModel::getCamModel()->width();

    for (auto tf : vTrackedFeatures) {
        if (!tf->m_bDead) {
            auto xy = tf->m_vVisualObs.back().m_px;
            _setMask(xy.x(), xy.y());
        }
    }
    //Todo: test mask correctness

    //Extract More Points out of mask
    int halfMaskSize = (m_nMaskSize - 1) / 2;

    std::vector<cv::Point2f> corners;

#if USE_HARRIS

    _extractHarris(corners, m_nMax2Track - m_nTracked);

#else
    _extractFast(imgStride, halfMaskSize, corners);

#endif

    for (int i = 0, j = m_nMax2Track - m_nTracked; i < corners.size() && j > 0; ++i) {
        int x = corners[i].x;
        int y = corners[i].y;
        assert(x + y * imgStride < m_nMaskBufferSize);
        if (m_pMask[x + y * imgStride]) {
            _setMask(x, y);
            j--;
            auto tf = std::make_shared<TrackedFeature>();
            tf->addVisualObservation(Vector2f(x, y), m_pCamState);
            vTrackedFeatures.push_back(tf);
            ++m_iFeature;
        }
    }
}

void FeatureTrackerOpticalFlow_Chen::_trackPoints(std::list<TrackedFeaturePtr>& vTrackedFeatures) {
    if (lastImage.empty())
        return;
    Matrix3f dR = m_pCamState->state->m_Rwi.transpose() * m_pCamState0->state->m_Rwi;

#if CV_MAJOR_VERSION == 3
    //        auto lkOpticalFlow =  cv::SparsePyrLKOpticalFlow::create(cv::Size(11,11),2,cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS,15,0.01));
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
        if (!tf->m_bDead) {
            auto& lastVisualOb = tf->m_vVisualObs.back();

#if USE_ROTATION_PREDICTION
            Vector3f ray = dR * lastVisualOb.m_Ray;

            Vector2f px = camModel->camToImage(ray);
            if (!camModel->inView(px)) {
                tf->m_bDead = true;
                continue;
            }
            now.emplace_back(px.x(), px.y());
            pre.emplace_back(lastVisualOb.m_px.x(), lastVisualOb.m_px.y());
            goodTracks.push_back(tf.get());
            tf->m_PredictedPx = px;
#else
#if 0
                Vector3f ray = dR * lastVisualOb.m_Ray;

                static Matrix3f Rci = camModel->getRci();
                Vector2f px = camModel->camToImage(Rci * ray);
                tf->m_PredictedPx = px;
#endif
            pre.emplace_back(lastVisualOb.m_px.x(), lastVisualOb.m_px.y());
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
    cv::calcOpticalFlowPyrLK(lastImage, m_image, pre, now, status, err, cv::Size(21, 21), 3, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01), use_predict ? cv::OPTFLOW_USE_INITIAL_FLOW | cv::OPTFLOW_LK_GET_MIN_EIGENVALS : 0,5e-3);

    for (int i = 0; i < status.size(); ++i) {
        Vector2f px;
        if (status[i]) {
            px.x() = now[i].x;
            px.y() = now[i].y;
            if (camModel->inView(px, 5)) {
                m_iFeature++;
                m_nTracked++;
                goodTracks[i]->addVisualObservation(px, m_pCamState);
                continue;
            }
        }

        goodTracks[i]->m_bDead = true;
    }

    int nRansac = DataAssociation::removeOutlierBy2PointRansac(dR, vTrackedFeatures);

    //LOGW("nPointsLast:%d nPointsTracked:%d nPointsAfterRansac:%d", pre.size(), m_nTracked, nRansac);
}

void FeatureTrackerOpticalFlow_Chen::matchNewFrame(std::list<TrackedFeaturePtr>& vTrackedFeatures, cv::Mat& image,
                                                   Frame* camState) {
    m_iFeature   = 0;
    m_image      = image;
    m_pCamState0 = m_pCamState;
    m_pCamState  = camState;

    //Set cnt for tracked points to zero
    m_nTracked = 0;

    //Init mask buffer
    if (m_pMask == nullptr) {
        m_nMaskBufferSize = CamModel::getCamModel()->area();
        m_pMask           = new unsigned char[m_nMaskBufferSize];
    }
    TickTock::start("KLT");
    _trackPoints(vTrackedFeatures);
    TickTock::stop("KLT");
    // Extract more points if there are more points can be tracked.
    if (m_nTracked < m_nMax2Track) {
        TickTock::start("Fast");
        _extractMorePoints(vTrackedFeatures);
        TickTock::stop("Fast");
    }

    lastImage = m_image;
}

FeatureTrackerOpticalFlow_Chen::~FeatureTrackerOpticalFlow_Chen() {
    delete[] m_pMask;
    m_pMask = nullptr;
}

void FeatureTrackerOpticalFlow_Chen::_extractFast(const int imgStride, const int halfMaskSize, std::vector<cv::Point2f>& corner) {
    std::vector<fast::fast_xy> vXys;
    std::vector<int> vScores, vNms;
    ;
    std::vector<std::pair<int, fast::fast_xy>> vTemp;
    fast::fast_corner_detect_10_mask(m_image.data + halfMaskSize + halfMaskSize * imgStride, m_pMask + halfMaskSize + halfMaskSize * imgStride, m_image.cols - m_nMaskSize + 1, m_image.rows - m_nMaskSize + 1, m_image.step1(), 15, vXys);
    fast::fast_corner_score_10(m_image.data + halfMaskSize + halfMaskSize * imgStride, m_image.step1(), vXys, 15, vScores);
    fast::fast_nonmax_3x3(vXys, vScores, vNms);

    vTemp.reserve(vXys.size());
    for (auto& idx : vNms) {
        vTemp.emplace_back(vScores[idx], vXys[idx]);
    }

#if USE_STABLE_SORT
    std::stable_sort(vTemp.begin(), vTemp.end(), [](const std::pair<int, fast::fast_xy>& a, const std::pair<int, fast::fast_xy>& b) { return a.first > b.first; });
#else

    std::sort(vTemp.begin(), vTemp.end(), [](auto& a, auto& b) { return a.first > b.first; });
#endif

    corner.reserve(vTemp.size());
    for (auto& v : vTemp) {
        corner.emplace_back(v.second.x + halfMaskSize, v.second.y + halfMaskSize);
    }
}

void FeatureTrackerOpticalFlow_Chen::_extractHarris(std::vector<cv::Point2f>& corners, int max_num) {
    cv::Mat mask(480, 640, CV_8UC1, m_pMask);
    cv::goodFeaturesToTrack(m_image, corners, max_num, 0.1, 20, mask);
}
}  // namespace DeltaVins
