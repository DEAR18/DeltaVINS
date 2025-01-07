#pragma once

#include "utils/typedefs.h"
#include <dataStructure/sensorStructure.h>


namespace DeltaVins
{
	struct Pose
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		long long timestamp;
		Matrix3f Rwb;

		Vector3f Pwb;//body position in world frame
		using Ptr = std::shared_ptr<Pose>;
	};



}