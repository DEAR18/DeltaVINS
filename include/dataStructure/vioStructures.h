#pragma once
#include <memory>
#include <unordered_set>

#include "utils/typedefs.h"
#include <vector>


#include "Algorithm/Nonliear_LM.h"
#include "filterStates.h"

namespace DeltaVins
{
	struct Frame;

	struct TrackedFeature;


    struct VisualObservation {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        VisualObservation(const Vector2f &px, const Vector3f &ray, Frame *frame) : m_px(px), m_Ray(ray), m_linkFrame(frame){}

        Vector2f        m_px_reprj;             //reprojected position only used for debug
        Vector2f		m_px;					//feature position
        Vector3f		m_Ray;					//camera ray
        Frame*			m_linkFrame = nullptr;	//pointer to linked frame


    };
	
	struct Frame
	{
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        Frame();

        ~Frame();

        void removeLinksFromAllTrackedFeatures(TrackedFeature* ftTrack);

        void removeAllFeatures();

        cv::Mat image;												//image captured from camera

        CamState* state= nullptr;									//pointer to camera states including position,rotation,etc.

        std::unordered_set<TrackedFeature*> m_vTrackedFeatures;		//All tracked feature in this frame.
        long long timestamp;


#if USE_KEYFRAME
        bool m_bKeyframe;;
#endif
		using Ptr = std::shared_ptr<Frame>;
	};
	
	struct TrackedFeature: public NonLinear_LM<3, float>
	{
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        TrackedFeature();

        ~TrackedFeature();

        bool	m_bDead;			//still be tracked
        int     m_iDeadFrameId;      //used for debug
        int		m_nObs;				//the number of observations matched
        float	m_rayAngle;			//ray angle between current camera ray and the first ray
        float	m_rayAngle0;		//last ray angle
        float	m_LastMovedPx;		//the pixel distance last frame moved
        int     m_id;               //used for debug
		
		
        std::vector<VisualObservation>	m_vVisualObs;		//vector of all visual observations
        Vector2f						m_PredictedPx;		//predicted pixel position using propagated camera pose
        PointState*						m_pState;			//pointer to point state
#if USE_NEW_MOVED_PIXEL||USE_POSITION_DETECT_ROTATION
		float   m_movedPx;          //max moved pixel
        bool    m_bInaccurateDepth;
#endif
#if USE_KEYFRAME
        Frame*                          m_pHostFrame;
        bool                            m_bSlamPointCandidate;
#endif
        std::vector<Matrix3f>			m_vdR;				//used in triangulate
        std::vector<Vector3f>			m_vdt;

        void addVisualObservation(const Vector2f& px, Frame* frame);
        void popObservation();

        void removeLinksInCamStates();
        void drawFeatureTrack(cv::Mat& image, cv::Scalar color) const;
        void reproject();
        void drawObservationsAndReprojection(int time=0);
        void printObservations();

        void removeUselessObservationForSlamPoint();

        //Triangulation
        bool userDefinedConvergeCriteria()override;
        void printPositions();
        bool triangulate();
        float evaluateF(bool bNewZ, float huberThresh) override;
        bool userDefinedDecentFail() override;
        using Ptr = std::shared_ptr<TrackedFeature>;

#if USE_DEPTH_PRIOR
        float mean_depth = -1;
        float info = -1;
#endif

    };


}
