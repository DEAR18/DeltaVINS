#pragma once
#include <memory>
#include "utils/typedefs.h"

namespace DeltaVins
{
    struct Frame;
    typedef std::shared_ptr<Frame> FramePtr;
    struct TrackedFeature;
    typedef std::shared_ptr<TrackedFeature> TrackedFeaturePtr;


    struct PointState{

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        Vector3f				m_Pw;				//	point position in world frame
        Vector3f				m_Pw_FEJ;			//	point position First Estimate Jacobian

        MatrixXfR			H;						//	Observation Matrix
        TrackedFeature*		host = nullptr;

        bool                m_bToMargin = false;
        bool                m_bNextMargin = false;
        bool                bSlamPoint;
        int m_idx;                                  // point idx in sliding window
    	
        int m_id = 0;								//only used in visualizer
        int m_idVis = -1;
        PointState()
        {
            static int counter = 0;
            m_id = counter++;
            bSlamPoint = false;
        }
    };


    struct CamState{
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        Matrix3f	m_Rwi;					// rotation matrix from imu frame to world frame
        Vector3f	m_Pwi;					// imu position in world frame
        Vector3f    m_Pw_FEJ;				// First Estimate Jacobian Imu Position in world frame

        int			m_idx;					// camera idx in sliding window
        bool		m_bToMargin = false;	// flag to marginalize
        Frame*		m_pHost = nullptr;		// pointer to host frame
        Vector3f    m_vel;


        int m_id = 0;						//only used in visualizer

#if USE_NEW_MOVED_PIXEL
        std::vector<Matrix3f>           m_dR;
#endif
    	
        CamState()
        {
            static int counter = 0;
            m_id = counter++;
        }
    };





    struct MsckfState {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        std::vector<FramePtr>			m_vFrames;	//All frames in sliding window
        Vector3f							vel;		// linear velocity
    };

}
