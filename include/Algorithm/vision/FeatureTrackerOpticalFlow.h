#pragma once
#include "dataStructure/vioStructures.h"

namespace DeltaVins
{
	class FeatureTrackerOpticalFlow
	{
	public:

		void trackFrame(const cv::Mat& image, Frame::Ptr& newFrame, std::list<TrackedFeature::Ptr>& featureLists);
		
	};
	
}
