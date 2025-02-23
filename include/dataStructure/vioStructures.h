#pragma once
#include <memory>
#include <opencv2/opencv.hpp>
#include <unordered_set>
#include <vector>

#include "Algorithm/Nonliear_LM.h"
#include "filterStates.h"
#include "utils/typedefs.h"

namespace DeltaVins {
struct Frame;

struct Landmark;

struct VisualObservation {
    using Ptr = std::shared_ptr<VisualObservation>;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VisualObservation(const Vector2f& px, Frame* frame, int cam_id);

    Vector2f px_reprj;            // reprojected position only used for debug
    Vector2f px;                  // feature position
    Vector3f ray_in_cam;          // camera ray
    Frame* link_frame = nullptr;  // pointer to linked frame
    Landmark* link_landmark = nullptr;  // pointer to linked landmark
    VisualObservation* stereo_obs =
        nullptr;  // pointer to the corresponding observation
    int cam_id;   // 0: left, 1: right
};

struct Frame {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Frame(int sensor_id);

    ~Frame();

    void RemoveAllObservations();
    VisualObservation::Ptr AddVisualObservation(const Vector2f& px, int cam_id);

    cv::Mat image;        // image captured from camera
    cv::Mat image_right;  // image captured from the right camera.

    int sensor_id;
    // int cam_id;    // left or right
    int frame_id;  // frame id

    CamState* state =
        nullptr;  // pointer to camera states including position,rotation,etc.

    std::unordered_set<VisualObservation::Ptr>
        visual_obs[2];  // All visual observations in this frame.
    // std::unordered_set<VisualObservation::Ptr>
    //     visual_obs_right;  // All visual observations in the right camera.
    int valid_landmark_num;
    int64_t timestamp;

    bool flag_keyframe;
    using Ptr = std::shared_ptr<Frame>;
};

struct Landmark : public NonLinear_LM<3, double> {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    struct VisualObservationComparator {
        bool operator()(const VisualObservation::Ptr& a,
                        const VisualObservation::Ptr& b) const {
            return a->link_frame->frame_id < b->link_frame->frame_id;
        }
    };

    Landmark();

    ~Landmark();

    int valid_obs_num;
    bool flag_dead_all;         // still be tracked
    bool flag_dead[2];          // still be tracked
    int flag_dead_frame_id[2];  // used for debug
    int num_obs_tracked;        // the number of observations matched
    float ray_angle;   // ray angle between current camera ray and the first ray
    float ray_angle0;  // last ray angle
    int landmark_id_;  // used for debug

    std::set<VisualObservation::Ptr, VisualObservationComparator>
        // std::unordered_set<VisualObservation::Ptr>
        visual_obs[2];                    // vector of all visual observations
    VisualObservation::Ptr last_obs_[2];  // pointer to the last observation
    VisualObservation::Ptr
        last_last_obs_[2];     // pointer to the second last observation
    Vector2f predicted_px[2];  // predicted pixel position using propagated
                               // camera pose
    PointState* point_state_;  // pointer to point state
    Frame* host_frame;
    bool flag_slam_point_candidate;
    std::vector<Matrix3d> dRs[2];  // used in Triangulate
    std::vector<Vector3d> dts[2];

    void SetDeadFlag(bool dead, int cam_id);  // -1: all, 0: left, 1: right

    void AddVisualObservation(VisualObservation::Ptr obs, int cam_id);
    void RemoveVisualObservation(VisualObservation::Ptr obs);
    void PopObservation(int cam_id);

    void RemoveLinksInCamStates();
    void DrawFeatureTrack(cv::Mat& image, cv::Scalar color,
                          int cam_id = 0) const;
    float Reproject(bool verbose = true, int cam_id = 0);
    void DrawObservationsAndReprojection(int time = 0, int cam_id = 0);
    void PrintObservations(int cam_id = 0);

    void RemoveUselessObservationForSlamPoint();

    // Triangulation
    bool UserDefinedConvergeCriteria() override;
    void PrintPositions();
    bool Triangulate();
    bool TriangulateLM(float depth_prior);
    bool TriangulationAnchorDepth(float& anchor_depth);
    bool StereoTriangulate();
    double EvaluateF(bool bNewZ, double huberThresh) override;
    bool UserDefinedDecentFail() override;
    using Ptr = std::shared_ptr<Landmark>;
};

}  // namespace DeltaVins
