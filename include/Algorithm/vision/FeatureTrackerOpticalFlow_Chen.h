#pragma once
#include "dataStructure/vioStructures.h"

namespace DeltaVins {
class FeatureTrackerOpticalFlow_Chen {
   public:
    FeatureTrackerOpticalFlow_Chen(int nMax2Track, int nMaskSize = 31);
    void _SetMask(int x, int y);

    void _ExtractMorePoints(std::list<TrackedFeaturePtr>& vTrackedFeatures);
    void _TrackPoints(std::list<TrackedFeaturePtr>& vTrackedFeatures);
    void MatchNewFrame(std::list<TrackedFeaturePtr>& vTrackedFeatures,
                       cv::Mat& image, Frame* camState);

    ~FeatureTrackerOpticalFlow_Chen();

   private:
    void _ExtractFast(const int imgStride, const int halfMaskSize,
                      std::vector<cv::Point2f>& vTemp);
    void _ExtractHarris(std::vector<cv::Point2f>& corners, int max_num);

    unsigned char* mask_ = nullptr;
    int num_features_;
    int max_num_to_track_;
    int mask_size_;
    int num_features_tracked_;
    int mask_buffer_size_;
    cv::Mat image_;
    Frame* cam_state_ = nullptr;
    Frame* cam_state0_ = nullptr;
    cv::Mat last_image_;
};

}  // namespace DeltaVins