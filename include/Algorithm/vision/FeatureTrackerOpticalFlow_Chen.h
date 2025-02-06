#pragma once
#include "dataStructure/vioStructures.h"
#include "dataStructure/sensorStructure.h"

namespace DeltaVins {
class FeatureTrackerOpticalFlow_Chen {
   public:
    FeatureTrackerOpticalFlow_Chen(int nMax2Track, int nMaskSize = 41);

    /**
     * @param vTrackedFeatures list of tracked features
     * @param image image
     * @param camState camera state
     */
    void MatchNewFrame(std::list<TrackedFeaturePtr>& vTrackedFeatures,
                     const ImageData::Ptr image, Frame* camState);

    ~FeatureTrackerOpticalFlow_Chen();

   private:
    void _ExtractMorePoints(std::list<TrackedFeaturePtr>& vTrackedFeatures);
    void _TrackPoints(std::list<TrackedFeaturePtr>& vTrackedFeatures);
    void _ExtractFast(const int imgStride, const int halfMaskSize,
                      std::vector<cv::Point2f>& vTemp);
    void _ExtractHarris(std::vector<cv::Point2f>& corners, int max_num);
    void _SetMask(int x, int y);
    bool _IsMasked(int x, int y);
    void _ResetMask();
    void _ShowMask();

    unsigned char* mask_ = nullptr;
    int num_features_;
    int max_num_to_track_;
    int mask_size_;
    int num_features_tracked_;
    int mask_buffer_size_;
    cv::Mat image_;
    std::vector<cv::Mat> image_pyramid_;
    cv::Mat right_image_;
    std::vector<cv::Mat> right_image_pyramid_;
    Frame* cam_state_ = nullptr;
    Frame* cam_state0_ = nullptr;
    cv::Mat last_image_;
    std::vector<cv::Mat> last_image_pyramid_;
};

}  // namespace DeltaVins