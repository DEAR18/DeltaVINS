#pragma once
#include <memory>
#include <unordered_set>
#include <vector>

#include "Algorithm/Nonliear_LM.h"
#include "filterStates.h"
#include "utils/typedefs.h"

namespace DeltaVins {
struct Frame;

struct TrackedFeature;

struct VisualObservation {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VisualObservation(const Vector2f& px, const Vector3f& ray, Frame* frame)
        : px(px), ray(ray), link_frame(frame) {}

    Vector2f px_reprj;           // reprojected position only used for debug
    Vector2f px;                 // feature position
    Vector3f ray;                // camera ray
    Frame* link_frame = nullptr;  // pointer to linked frame
};

struct Frame {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Frame();

    ~Frame();

    void RemoveLinksFromAllTrackedFeatures(TrackedFeature* ftTrack);

    void RemoveAllFeatures();

    cv::Mat image;  // image captured from camera

    CamState* state =
        nullptr;  // pointer to camera states including position,rotation,etc.

    std::unordered_set<TrackedFeature*>
        tracked_features;  // All tracked feature in this frame.
    long long timestamp;

#if USE_KEYFRAME
    bool flag_keyframe;
#endif
    using Ptr = std::shared_ptr<Frame>;
};

struct TrackedFeature : public NonLinear_LM<3, float> {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    TrackedFeature();

    ~TrackedFeature();

    bool flag_dead;        // still be tracked
    int flag_dead_frame_id;  // used for debug
    int num_obs;          // the number of observations matched
    float ray_angle;  // ray angle between current camera ray and the first ray
    float ray_angle0;    // last ray angle
    float last_moved_px;  // the pixel distance last frame moved
    int m_id;             // used for debug
    Vector3f Pw_stereo_prior;
    bool has_stereo_prior;

    std::vector<VisualObservation>
        visual_obs;  // vector of all visual observations
    Vector2f
        predicted_px;  // predicted pixel position using propagated camera pose
    PointState* point_state_;  // pointer to point state
#if USE_NEW_MOVED_PIXEL || USE_POSITION_DETECT_ROTATION
    float m_movedPx;  // max moved pixel
    bool m_bInaccurateDepth;
#endif
#if USE_KEYFRAME
    Frame* host_frame;
    bool flag_slam_point_candidate;
#endif
    std::vector<Matrix3f> dRs;  // used in Triangulate
    std::vector<Vector3f> dts;

    void AddVisualObservation(const Vector2f& px, Frame* frame);
    void AddVisualObservation(const Vector2f& px, Frame* frame,float depth);
    void PopObservation();

    void RemoveLinksInCamStates();
    void DrawFeatureTrack(cv::Mat& image, cv::Scalar color) const;
    void Reproject();
    void DrawObservationsAndReprojection(int time = 0);
    void PrintObservations();

    void RemoveUselessObservationForSlamPoint();

    // Triangulation
    bool UserDefinedConvergeCriteria() override;
    void PrintPositions();
    bool Triangulate();
    bool StereoTriangulate();
    float EvaluateF(bool bNewZ, float huberThresh) override;
    bool UserDefinedDecentFail() override;
    using Ptr = std::shared_ptr<TrackedFeature>;

#if USE_DEPTH_PRIOR
    float mean_depth = -1;
    float info = -1;
#endif
};

}  // namespace DeltaVins
