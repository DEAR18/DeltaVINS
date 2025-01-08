#pragma once
#include "dataStructure/vioStructures.h"

namespace DeltaVins {
class FeatureTrackerOpticalFlow_Chen {
   public:
    FeatureTrackerOpticalFlow_Chen(int nMax2Track, int nMaskSize = 31);
    void _setMask(int x, int y);

    void _extractMorePoints(std::list<TrackedFeaturePtr>& vTrackedFeatures);
    void _trackPoints(std::list<TrackedFeaturePtr>& vTrackedFeatures);
    void matchNewFrame(std::list<TrackedFeaturePtr>& vTrackedFeatures,
                       cv::Mat& image, Frame* camState);

    ~FeatureTrackerOpticalFlow_Chen();

   private:
    void _extractFast(const int imgStride, const int halfMaskSize,
                      std::vector<cv::Point2f>& vTemp);
    void _extractHarris(std::vector<cv::Point2f>& corners, int max_num);

    unsigned char* m_pMask = nullptr;
    int m_iFeature;
    int m_nMax2Track;
    int m_nMaskSize;
    int m_nTracked;
    int m_nMaskBufferSize;
    cv::Mat m_image;
    Frame* m_pCamState = nullptr;
    Frame* m_pCamState0 = nullptr;
    cv::Mat lastImage;
};

}  // namespace DeltaVins