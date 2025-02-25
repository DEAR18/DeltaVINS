#pragma once
#include <opencv2/opencv.hpp>
#include "dataStructure/IO_Structures.h"

namespace DeltaVins {

class StaticInitializer {
   public:
    StaticInitializer();
    ~StaticInitializer();

    bool Initialize(const ImageData::Ptr imageData);

   private:
    void _TrackFrame(const ImageData::Ptr imageData);

    void _Initialization(const ImageData::Ptr imageData);

    std::vector<cv::Point2f> last_points_;
    cv::Mat last_image_;

    float moved_pixels_last_few_frames_ = 3.0f;
};

}  // namespace DeltaVins