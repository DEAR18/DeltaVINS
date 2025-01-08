#include "Algorithm/vision/FeatureTrackerOpticalFlow.h"

#include "precompile.h"

namespace DeltaVins {
void FeatureTrackerOpticalFlow::trackFrame(
    const cv::Mat& image, Frame::Ptr& newFrame,
    std::list<TrackedFeature::Ptr>& featureLists) {}
}  // namespace DeltaVins
