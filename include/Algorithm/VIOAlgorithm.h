#pragma once

#include <FrameAdapter.h>
#include <WorldPointAdapter.h>

#include "IMU/ImuPreintergration.h"
#include "dataStructure/IO_Structures.h"
#include "solver/SquareRootEKFSolver.h"
#include "vision/FeatureTrackerOpticalFlow.h"
#include "vision/FeatureTrackerOpticalFlow_Chen.h"
namespace DeltaVins {
class VIOAlgorithm {
   public:
    VIOAlgorithm();
    ~VIOAlgorithm();

    void AddNewFrame(const ImageData::Ptr imageData, Pose::Ptr pose);

    void SetWorldPointAdapter(WorldPointAdapter* adapter);
    void SetFrameAdapter(FrameAdapter* adapter);

   private:
    struct SystemStates {
        Vector3f vel;
        std::vector<Frame::Ptr> frames_;
        std::list<TrackedFeature::Ptr> tfs_;
        bool static_;
#if USE_PLANE_PRIOR
        Vector3f m_PlaneCoeff;
        Vector3f n;
        //  (exp(m_Riw)* n).dot(Pw) + d = 0
#endif
    };

    void _PreProcess(const ImageData::Ptr imageData);
    void _PostProcess(ImageData::Ptr data, Pose::Ptr pose);
    void _UpdatePointsAndCamsToVisualizer();
    void _DrawTrackImage(ImageData::Ptr dataPtr, cv::Mat& trackImage);
    void _DrawPredictImage(ImageData::Ptr dataPtr, cv::Mat& predictImage);
    void Initialize(const Matrix3f& Rwi);

    void _AddImuInformation();
    void _RemoveDeadFeatures();
    void _MarginFrames();
    void _StackInformationFactorMatrix();
    void _DetectStill();
    void _TestVisionModule(const ImageData::Ptr data, Pose::Ptr pose);
    void _AddMeasurement();
    void _SelectFrames2Margin();

    bool _VisionStatic();

    FeatureTrackerOpticalFlow_Chen* feature_trakcer_ = nullptr;
    SquareRootEKFSolver* solver_ = nullptr;
    SystemStates states_;
    Frame::Ptr frame_now_ = nullptr;

    ImuPreintergration preintergration_;

    bool initialized_;

#if USE_KEYFRAME
    Frame::Ptr last_keyframe_ = nullptr;
    void _SelectKeyframe();
#endif

    /************* Output **********************/

    FrameAdapter* frame_adapter_ = nullptr;
    WorldPointAdapter* world_point_adapter_ = nullptr;
};

}  // namespace DeltaVins
