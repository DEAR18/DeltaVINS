#include "precompile.h"
#include "dataStructure/vioStructures.h"

#include "Algorithm/vision/camModel/camModel.h"
#include "utils/utils.h"

namespace DeltaVins
{
    Frame::Frame()
    {
        state = new CamState();
        state->m_pHost = this;
#if USE_KEYFRAME
        m_bKeyframe = false;
#endif
    }

    Frame::~Frame()
    {
        if (state) delete state;
    }

    void Frame::removeLinksFromAllTrackedFeatures(TrackedFeature* ftTrack)
    {
        m_vTrackedFeatures.erase(ftTrack);
    }

    void Frame::removeAllFeatures()
    {
        for(auto feature:m_vTrackedFeatures){
#if USE_KEYFRAME
            if (feature->m_pHostFrame == this)
                feature->m_pHostFrame = nullptr;
#endif
            for(auto&observation:feature->m_vVisualObs){
                if (observation.m_linkFrame == this) {
                    int nSize = feature->m_vVisualObs.size();
                    if (!feature->m_bDead)
                    {
                        observation = feature->m_vVisualObs[nSize - 2];
                        feature->m_vVisualObs[nSize - 2] = feature->m_vVisualObs[nSize - 1];
                    }

                    else
                        observation = feature->m_vVisualObs.back();
                    feature->m_vVisualObs.pop_back();
                    break;
                }
            }
        }
        m_vTrackedFeatures.clear();
    }


    TrackedFeature::~TrackedFeature()
    {
        if (!m_vVisualObs.empty())
            removeLinksInCamStates();
        if (m_pState)
        {
            delete m_pState;
            m_pState = nullptr;
        }
    }

    TrackedFeature::TrackedFeature():NonLinear_LM(1e-2,0.005,1e-3,15, false)
    {
        m_bDead = false;
        m_nObs = 0;
        m_rayAngle = 0;
        m_LastMovedPx = 0;
        m_pState = nullptr;
#if USE_KEYFRAME
        m_bSlamPointCandidate = 0;
        m_pHostFrame = nullptr;
#endif

#if USE_NEW_MOVED_PIXEL
        m_movedPx = 0;
        m_bInaccurateDepth = false;
#endif

        static int counter = 0;
        m_id = counter++;
    }

    bool TrackedFeature::triangulate()
    {

#if USE_NEW_MOVED_PIXEL

        static CamModel* camModel = CamModel::getCamModel();

        float max_MovedPx = 0;
        float max_MovedPx2 = 0;
        float min_px_thresh = 5 * 5;
        for (int i = 0, length = m_vVisualObs.size() - 1; i < length; ++i)
        {
            auto& ob = m_vVisualObs[i];
            auto frame = ob.m_linkFrame->state;
			for (int j=i+1;j<length;j++)
			{
                auto& ob2 = m_vVisualObs[j];
                //int idx = ob2.m_linkFrame->state->m_idx;
                //Vector2f px_project = camModel->camToImage(frame->m_dR[idx] * ob.m_Ray);
                //Vector2f px2 = ob2.m_px;
                //float px_moved = (px_project - ob2.m_px).squaredNorm();
                float px_moved = (ob2.m_px - ob.m_px).squaredNorm();
                if (px_moved > max_MovedPx)
                    max_MovedPx = px_moved;
				
			}


        }

        if (max_MovedPx < min_px_thresh)
            m_bInaccurateDepth = true;



#endif


#if USE_POSITION_DETECT_ROTATION

        float max_position = 0;
        float position_thresh = 0.05 * 0.05;
    	for (int i=0,n=m_vVisualObs.size();i<n;++i)
    	{
            const Vector3f& dP = m_vVisualObs[i].m_linkFrame->state->m_Pwi;
            for (int j=i+1;j<n;++j)
            {
                float dP2 = (dP - m_vVisualObs[j].m_linkFrame->state->m_Pwi).squaredNorm();
                if (dP2>max_position)
                {
                    max_position = dP2;
                }
            }

    		
    	}

    	if(max_position<position_thresh)
    	{

            m_bInaccurateDepth = true;
    		
    	}
        
#endif
    	
    	
        if(m_verbose)
            LOGI("###PointID:%d", m_id);
        //if (m_pState) return m_Result.bConverged;
        static Vector3f Tci = CamModel::getCamModel()->getTci();

        clear();

        auto& leftVisualOb = m_vVisualObs.front();
        //Vector3f pInImu = Rci.transpose() * (leftVisualOb.m_Ray_cam*2) + Pic;
        z = leftVisualOb.m_Ray;
        z /= z[2];
        z[2] = 1 / 2.f;
        /*z.z() = 1/pInImu.z();*/

        const int nSize = m_vVisualObs.size();
        m_vdR.resize(nSize);
        m_vdt.resize(nSize);
        Matrix3f leftR = leftVisualOb.m_linkFrame->state->m_Rwi;
        Vector3f leftP = leftVisualOb.m_linkFrame->state->m_Pwi;
        {
            for (int i = 0; i < nSize; ++i)
            {
                auto& ob = m_vVisualObs[i];
                Matrix3f R_i = ob.m_linkFrame->state->m_Rwi;
                Vector3f P_i = ob.m_linkFrame->state->m_Pwi;
                m_vdR[i] =  R_i.transpose() * leftR;
                m_vdt[i] =  R_i.transpose() * (leftP - P_i) + Tci - m_vdR[i] * Tci;

            }
        }

        solve();

        if (m_pState == nullptr) {
            m_pState = new PointState();
            m_pState->host = this;
        }
        Vector3f cpt = z / z[2];
        cpt[2] = 1.0f / z[2];
        m_pState->m_Pw = leftR * (cpt-Tci) + leftP;
        m_pState->m_Pw_FEJ = m_pState->m_Pw;
#if USE_KEYFRAME
        float depthRatio = 0.01;
        if (m_Result.bConverged && !m_bDead && m_Result.cost < 2 &&  H(2, 2) > depthRatio * H(0, 0) && H(2, 2) > depthRatio * H(1, 1))
            m_bSlamPointCandidate = true;

        if(z[2]<0.1)
            m_bSlamPointCandidate = false;

#endif

#if (USE_NEW_MOVED_PIXEL||USE_POSITION_DETECT_ROTATION)&&USE_KEYFRAME

        if (m_bInaccurateDepth) {
            m_bSlamPointCandidate = false;
        }
#endif

#if USE_DEPTH_PRIOR
		if(m_Result.bConverged)
		{
			if(mean_depth <0)
			{
                mean_depth = z[2];
                info = H(2, 2);
			}
            else
            {
                float alpha = info;
                float beta = H(2, 2);
                float s, c;
                if (fabs(beta) < FLT_EPSILON)
                {
						
                }
                else if (fabs(beta) > fabs(alpha)) {
                    s = 1 / sqrt(1 + pow(alpha / beta, 2));
                    c = -alpha / beta * s;
                }
                else {
                    c = 1 / sqrt(1 + pow(beta / alpha, 2));
                    s = -beta / alpha * c;
                }
                Matrix2f A;
                A << info, 0, beta, -beta * (mean_depth - z[2]);

            	A(1,0) = s * info + c * beta;
                A(1, 1) = c * A(1, 1);
            	
            	
            }
		}
#endif
        m_vdR.clear();
        m_vdt.clear();
        return m_Result.bConverged;
    }


    float TrackedFeature::evaluateF(bool bNewZ, float huberThresh)
    {
        float cost = 0.f;
        const int nSize = m_vVisualObs.size();
        Matrix23f J23;
        Matrix3f J33;
        Vector3f position;

        float huberCutTh = 10.f;

        if(bNewZ)
        {
            position = zNew / zNew[2];
            position[2] = 1.0 / zNew[2];
        }
        else {
            position = z / z[2];
            position[2] = 1.0 / z[2];

        }
        HTemp.setZero();
        bTemp.setZero();
        J33 <<	position[2], 0,				-position[0] * position[2],
                0,			position[2],	-position[1] * position[2],
                0,			 0,				-position[2] * position[2];
        for (int i = 0; i < nSize; ++i)
        {
            Vector3f p_cam = m_vdR[i] * position + m_vdt[i];

            m_vVisualObs[i].m_px_reprj = CamModel::getCamModel()->camToImage(p_cam, J23);


            Vector2f r = m_vVisualObs[i].m_px - m_vVisualObs[i].m_px_reprj;
#if HUBER||1
            float reprojErr = r.norm();
            float hw = reprojErr > huberThresh ? huberThresh / reprojErr : 1;
            //cost += reprojErr>huberThresh ? huberThresh*(2*reprojErr-huberThresh):reprojErr*reprojErr;
            if (reprojErr > huberThresh)
            {
                cost += huberThresh * (2 * reprojErr - huberThresh);
            }
            else
                cost += reprojErr * reprojErr;
#else
            float hw = 1.0f;
			cost += r.squaredNorm();
#endif

            Matrix23f J = J23 * m_vdR[i] * J33;

            HTemp.noalias() += J.transpose() * J *hw;
            bTemp.noalias() += J.transpose() * r *hw;

        }
        //if(!bNewZ)
        //	H += Matrix3f::Identity()*FLT_EPSILON;
        return cost;
    }

    bool TrackedFeature::userDefinedDecentFail()
    {
        return zNew[2] < 0;
    }


    void TrackedFeature::addVisualObservation(const Vector2f& px, Frame* frame)
    {
        assert(CamModel::getCamModel()->inView(px));
        m_bDead = false;

        if(!m_vVisualObs.empty()){
            m_LastMovedPx = (m_vVisualObs.back().m_px - px).squaredNorm();
        }
        m_nObs++;

#if USE_KEYFRAME

    	if(m_pState&& m_pState->bSlamPoint)
    	{
            int n = m_vVisualObs.size();
            if (n > 2) {
                auto& obs_3 = m_vVisualObs[n - 3];
                if (!obs_3.m_linkFrame->m_bKeyframe)
                {
                    obs_3.m_linkFrame->m_vTrackedFeatures.erase(this);
                    obs_3 = m_vVisualObs[n - 2];
                    m_vVisualObs[n - 2] = m_vVisualObs[n - 1];
                    m_vVisualObs.pop_back();
                }
            }
    	}

#endif

        Vector3f ray0 = CamModel::getCamModel()->imageToImu(px);

        m_vVisualObs.emplace_back(px, ray0, frame);

        auto &rightOb = m_vVisualObs.back();
         

        frame->m_vTrackedFeatures.insert(this);

        m_rayAngle0 = m_rayAngle;
        Vector3f ray1 = rightOb.m_linkFrame->state->m_Rwi*rightOb.m_Ray;

        float minDot = 2;
        for (size_t i = 0, length = m_vVisualObs.size() - 1; i < length; i++)
        {
            auto&visualOb = m_vVisualObs[i];
            float dot = (visualOb.m_linkFrame->state->m_Rwi*visualOb.m_Ray).dot(ray1);
            if (dot < minDot)
                minDot = dot;
        }
        if (minDot > 0 && minDot < 1) {
            m_rayAngle = std::max(m_rayAngle, acosf(minDot));
        }

    }

    void TrackedFeature::drawFeatureTrack(cv::Mat& image, cv::Scalar color)const
    {
        for (int i=0;i<m_vVisualObs.size()-1;++i)
        {
            if((m_vVisualObs[i].m_px - m_vVisualObs[i+1].m_px).squaredNorm()>900)
                continue;
            cv::line(image,cv::Point(m_vVisualObs[i].m_px.x(),m_vVisualObs[i].m_px.y()),cv::Point(m_vVisualObs[i+1].m_px.x(),m_vVisualObs[i+1].m_px.y()),_GREEN_SCALAR,1);
            cv::circle(image,cv::Point(m_vVisualObs[i].m_px.x(),m_vVisualObs[i].m_px.y()),2, color);
        }
        cv::circle(image,cv::Point(m_vVisualObs.back().m_px.x(),m_vVisualObs.back().m_px.y()),8, color);
    }

    void TrackedFeature::reproject()
    {
        assert(m_pState);
        auto camModel = CamModel::getCamModel();
        float reprojErr = 0;
        for (auto &ob:m_vVisualObs)
        {
            ob.m_px_reprj = camModel->imuToImage(ob.m_linkFrame->state->m_Rwi.transpose() * (m_pState->m_Pw - ob.m_linkFrame->state->m_Pwi));
            reprojErr += (ob.m_px_reprj - ob.m_px).norm();
        }
        reprojErr /= m_vVisualObs.size();
        if(reprojErr>3)
        {
            if (reprojErr > 5)
                //drawObservationsAndReprojection();
                printf("Reproj Err:%f, Triangulation Cost:%f\n", reprojErr, m_Result.cost);
        }
    }

    void TrackedFeature::drawObservationsAndReprojection(int time)
    {
#if ENABLE_VISUALIZER && !defined(PLATFORM_ARM)
        if (Config::NoGUI) return;
        cv::Mat display;
        bool first = 1;
        for (auto&ob:m_vVisualObs)
        {
            cv::cvtColor(ob.m_linkFrame->image, display, CV_GRAY2BGR);
            if (first) {
                cv::circle(display, cv::Point(ob.m_px.x(), ob.m_px.y()), 8, _GREEN_SCALAR);
                cv::circle(display, cv::Point(ob.m_px_reprj.x(), ob.m_px_reprj.y()), 10, _BLUE_SCALAR);
                first = 0;
            }
         else
         {
                cv::circle(display, cv::Point(ob.m_px.x(), ob.m_px.y()), 4, _GREEN_SCALAR);
                cv::circle(display, cv::Point(ob.m_px_reprj.x(), ob.m_px_reprj.y()), 6, _BLUE_SCALAR);
         }
            cv::imshow("ob and reproj", display);
            cv::waitKey(time);
        }
#endif

    }

    void TrackedFeature::printObservations()
    {
        for(int i=0;i<m_vVisualObs.size();++i)
        {
            LOGI("Idx:%d,Px: %f %f,Pos:%f %f %f", i, m_vVisualObs[i].m_px.x(), m_vVisualObs[i].m_px.y(),m_vVisualObs[i].m_linkFrame->state->m_Pwi.x(),m_vVisualObs[i].m_linkFrame->state->m_Pwi.y(), m_vVisualObs[i].m_linkFrame->state->m_Pwi.z());
        }
    }
#if USE_KEYFRAME
    void TrackedFeature::removeUselessObservationForSlamPoint()
    {
        assert(m_pState && m_pState->bSlamPoint);
        std::vector<VisualObservation> obs;
		for(int i=0,n=m_vVisualObs.size();i<n;++i)
    	{
            auto& ob = m_vVisualObs[i];
    		if(ob.m_linkFrame->m_bKeyframe||i>=n-2)
    		{
                obs.push_back(ob);
    		}else
    		{
                ob.m_linkFrame->m_vTrackedFeatures.erase(this);
    		}
    	}
    	
        m_vVisualObs = obs;
    }
#endif
    bool TrackedFeature::userDefinedConvergeCriteria()
    {
        if (m_Result.cost < 10)
            m_Result.bConverged = true;
        if (m_Result.cost > 40)
            m_Result.bConverged = false;
        return true;
    }

    void TrackedFeature::printPositions()
    {
    }

    void TrackedFeature::popObservation()
    {
        m_vVisualObs.back().m_linkFrame->removeLinksFromAllTrackedFeatures(this);
        m_rayAngle = m_rayAngle0;
        m_vVisualObs.pop_back();
    }

    void TrackedFeature::removeLinksInCamStates(){
        for (auto&ob : m_vVisualObs)
            ob.m_linkFrame->removeLinksFromAllTrackedFeatures(this);
        m_vVisualObs.clear();
    }


}
