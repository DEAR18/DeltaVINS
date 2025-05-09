#pragma once
#include "dataStructure/sensorStructure.h"
#include "dataStructure/vioStructures.h"

namespace DeltaVins {
class FeatureTrackerOpticalFlow_Chen {
   public:
    FeatureTrackerOpticalFlow_Chen(int nMax2Track, int nMaskSize = 41);

    /**
     * @param vTrackedFeatures list of tracked features
     * @param image image
     * @param camState camera state
     */
    void MatchNewFrame(std::list<LandmarkPtr>& vTrackedFeatures,
                       const ImageData::Ptr image, Frame* camState);

    bool IsStaticLastFrame();
    ~FeatureTrackerOpticalFlow_Chen();

   private:
    void _PreProcess(const ImageData::Ptr image, Frame* camState);
    void _PostProcess(std::list<LandmarkPtr>& vTrackedFeatures);
    void _ExtractMorePoints(std::list<LandmarkPtr>& vTrackedFeatures);
    void _TrackPoints(std::list<LandmarkPtr>& vTrackedFeatures);
    void _ExtractFast(const int imgStride, const int halfMaskSize,
                      std::vector<cv::Point2f>& vTemp, int cam_id);
    void _ExtractHarris(std::vector<cv::Point2f>& corners, int max_num,
                        int cam_id);
    void _SetMask(int x, int y, int cam_id);
    bool _IsMasked(int x, int y, int cam_id);
    void _ResetMask();
    void _ShowMask();

    void _TrackFromLastFrame(std::list<LandmarkPtr>& vTrackedFeatures,
                             int cam_id);
    void _TrackStereoFeatures(std::list<LandmarkPtr>& vTrackedFeatures);

    unsigned char* mask_ = nullptr;
    unsigned char* mask_right_ = nullptr;
    int num_features_;
    int max_num_to_track_;
    int mask_size_;
    int num_features_tracked_;
    int mask_buffer_size_;
    bool use_back_tracking_;
    // cv::Mat image_;
    std::vector<cv::Mat> image_pyramid_[2];
    Frame* cam_state_ = nullptr;
    Frame* cam_state0_ = nullptr;
    // cv::Mat last_image_;
    std::vector<cv::Mat> last_image_pyramid_[2];

    ImageData::Ptr image_;
    ImageData::Ptr last_image_;

    std::vector<float> last_frame_moved_pixels_sqr_;
};

}  // namespace DeltaVins