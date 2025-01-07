#pragma once


#include "dataStructure/IO_Structures.h"
#include "IMU/ImuPreintergration.h"
#include "solver/SquareRootEKFSolver.h"
#include "vision/FeatureTrackerOpticalFlow.h"
#include "vision/FeatureTrackerOpticalFlow_Chen.h"

#include <WorldPointAdapter.h>
#include <FrameAdapter.h>
namespace DeltaVins
{
	class VIOAlgorithm
	{
	public:


		VIOAlgorithm();
		~VIOAlgorithm();



		
		void addNewFrame(const ImageData::Ptr imageData, Pose::Ptr pose);

		

		void setWorldPointAdapter(WorldPointAdapter* adapter);
		void setFrameAdapter(FrameAdapter* adapter);

	private:

		struct SystemStates
		{
			Vector3f vel;
			std::vector<Frame::Ptr> mv_frames;
			std::list<TrackedFeature::Ptr> ml_tfs;
			bool bStatic;
#if USE_PLANE_PRIOR
			Vector3f m_PlaneCoeff;
			Vector3f n;
			//  (exp(m_Riw)* n).dot(Pw) + d = 0
#endif
			
		};
		
		
		void _preProcess(const ImageData::Ptr imageData);
		void _postProcess(ImageData::Ptr data, Pose::Ptr pose);
		void _updatePointsAndCamsToVisualizer();
		void _drawTrackImage(ImageData::Ptr dataPtr, cv::Mat& trackImage);
		void _drawPredictImage(ImageData::Ptr dataPtr, cv::Mat& predictImage);
		void initialize(const Matrix3f& Rwi);

		void _addImuInformation();
		void _removeDeadFeatures();
		void _marginFrames();
		void _stackInformationFactorMatrix();
		void _DetectStill();
		void _testVisionModule(const ImageData::Ptr data, Pose::Ptr pose);
		void _addMeasurement();
		void _selectFrames2Margin();

		bool _visionStatic();
		
		FeatureTrackerOpticalFlow_Chen* mp_featureTracker =nullptr;
		SquareRootEKFSolver* mp_solver = nullptr;
		SystemStates m_states;
		Frame::Ptr mp_frameNow = nullptr;

		ImuPreintergration m_preintergration;

		bool mb_Initialized;

#if USE_KEYFRAME
		Frame::Ptr  mp_lastKeyframe = nullptr;
		void _selectKeyframe();
#endif



		/************* Output **********************/

		FrameAdapter* mp_FrameAdapter = nullptr;
		WorldPointAdapter* mp_WorldPointAdapter = nullptr;

	};

	
}
