#include "precompile.h"
#include "Algorithm/DataAssociation/DataAssociation.h"


#include "Algorithm/DataAssociation/TwoPointRansac.h"
#include "Algorithm/solver/SquareRootEKFSolver.h"
#include "Algorithm/vision/camModel/camModel.h"
#include "dataStructure/vioStructures.h"
#include "utils/utils.h"


#define MAX_MSCKF_FEATURE_UPDATE_PER_FRAME MAX_POINT_SIZE

namespace DeltaVins
{
    namespace DataAssociation
    {

        TwoPointRansac* m_pTwoPointRansac = nullptr;
        SquareRootEKFSolver* m_pSquareRootSolver = nullptr;
        std::vector<TrackedFeaturePtr> m_vTrackedFeatureToUpdate;
        std::vector<TrackedFeaturePtr> m_vTrackedFeatureNextUpdate;
        std::vector<std::vector<TrackedFeaturePtr>> m_Grid22;


        cv::Mat reprojImage;
        cv::Mat reprojImage2;

        void clear()
        {
            m_vTrackedFeatureNextUpdate.clear();
            m_vTrackedFeatureToUpdate.clear();
            delete m_pTwoPointRansac;
        }

        void DrawPointsAfterUpdates(std::vector<PointState*>& m_PointStates)
        {

            if (Config::NoGUI) return;
            reprojImage2 = cv::Mat::zeros(480, 640, CV_8UC3);
            for (auto&p:m_vTrackedFeatureToUpdate)
            {
                p->reproject();
                for (auto& ob : p->m_vVisualObs) {
                    cv::circle(reprojImage2, cv::Point(ob.m_px.x(), ob.m_px.y()), 4, _BLUE_SCALAR);
                    cv::circle(reprojImage2, cv::Point(ob.m_px_reprj.x(), ob.m_px_reprj.y()), 4, _RED_SCALAR);
                    cv::line(reprojImage2, cv::Point(ob.m_px.x(), ob.m_px.y()), cv::Point(ob.m_px_reprj.x(), ob.m_px_reprj.y()), _GREEN_SCALAR);
                }
            	for(int i=0;i<p->m_vVisualObs.size()-1;++i)
            	{
                    cv::line(reprojImage2, cv::Point(p->m_vVisualObs[i].m_px.x(), p->m_vVisualObs[i].m_px.y()), cv::Point(p->m_vVisualObs[i+1].m_px.x(), p->m_vVisualObs[i + 1].m_px.y()), _GREEN_SCALAR);
            	}
            }
            for (auto& p2 : m_PointStates)
            {
                auto p = p2->host;
                p->reproject();
                auto& ob = p->m_vVisualObs.back();
                cv::circle(reprojImage2, cv::Point(ob.m_px.x(), ob.m_px.y()), 10, _BLUE_SCALAR);
                cv::circle(reprojImage2, cv::Point(ob.m_px_reprj.x(), ob.m_px_reprj.y()), 10, _RED_SCALAR);
                cv::line(reprojImage2, cv::Point(ob.m_px.x(), ob.m_px.y()), cv::Point(ob.m_px_reprj.x(), ob.m_px_reprj.y()), _GREEN_SCALAR);
            }
            cv::imshow("Points After Updates", reprojImage2);
        }
        void DrawPointsBeforeUpdates(std::vector<PointState*>& m_PointStates)
        {
            if (Config::NoGUI) return;
            reprojImage = cv::Mat::zeros(480, 640, CV_8UC3);

            for (auto& p : m_vTrackedFeatureToUpdate)
            {
                p->reproject();
                for (auto& ob : p->m_vVisualObs) {
                    cv::circle(reprojImage, cv::Point(ob.m_px.x(), ob.m_px.y()), 4, _BLUE_SCALAR);
                    cv::circle(reprojImage, cv::Point(ob.m_px_reprj.x(), ob.m_px_reprj.y()), 4, _RED_SCALAR);
                    cv::line(reprojImage, cv::Point(ob.m_px.x(), ob.m_px.y()), cv::Point(ob.m_px_reprj.x(), ob.m_px_reprj.y()), _GREEN_SCALAR);
                }
                for (int i = 0; i < p->m_vVisualObs.size() - 1; ++i)
                {
                    cv::line(reprojImage, cv::Point(p->m_vVisualObs[i].m_px.x(), p->m_vVisualObs[i].m_px.y()), cv::Point(p->m_vVisualObs[i + 1].m_px.x(), p->m_vVisualObs[i + 1].m_px.y()), _GREEN_SCALAR);
                }
            }

            for (auto& p2 : m_PointStates)
            {
                auto p = p2->host;
                p->reproject();
                auto& ob = p->m_vVisualObs.back();
                cv::circle(reprojImage, cv::Point(ob.m_px.x(), ob.m_px.y()), 10, _BLUE_SCALAR);
                cv::circle(reprojImage, cv::Point(ob.m_px_reprj.x(), ob.m_px_reprj.y()), 10, _RED_SCALAR);
                cv::line(reprojImage, cv::Point(ob.m_px.x(), ob.m_px.y()), cv::Point(ob.m_px_reprj.x(), ob.m_px_reprj.y()), _GREEN_SCALAR);


            }
        	cv::imshow("Points Before Updates", reprojImage);
        }


        void initDataAssociation(SquareRootEKFSolver* solver)
        {
            if (!m_pTwoPointRansac)
                m_pTwoPointRansac = new TwoPointRansac();
            m_pSquareRootSolver = solver;
            m_Grid22.resize(4);
        }

        int removeOutlierBy2PointRansac(Matrix3f& dR, std::list<TrackedFeaturePtr>& vTrackedFeatures)
        {
            assert(m_pTwoPointRansac);

            std::vector<Vector3f> ray0, ray1;
            std::vector<Vector2f> p0, p1;
            std::vector<TrackedFeature*> goodTracks;
            ray0.reserve(600);
            ray1.reserve(600);
            p0.reserve(600);
            p1.reserve(600);
            goodTracks.reserve(600);
            int nGoodPoints = 0;
#if NEW_TWO_POINT
            std::unordered_map<Frame*, std::pair<Matrix3f,std::vector<TrackedFeaturePtr>>> vRealtiveSet;

			auto t = std::find_if(vTrackedFeatures.begin(), vTrackedFeatures.end(), [](auto& a) {return !a->m_bDead; });
			Frame* frame = (*t)->m_vVisualObs.back().m_linkFrame;

			for(const auto&trackedFeature:vTrackedFeatures)
			{
				Frame* frame0 = trackedFeature->m_vVisualObs.front().m_linkFrame;
				if(frame0 != frame)
				{
					if(!vRealtiveSet.count(frame0))
						vRealtiveSet[frame0].first = frame->state->m_Rwi.transpose() * frame0->state->m_Rwi;
					vRealtiveSet[frame0].second.push_back(trackedFeature);
				}
			}

			for(auto&pair:vRealtiveSet)
			{
				ray0.clear();
				ray1.clear();
				p0.clear();
				p1.clear();
				goodTracks.clear();
				for (const auto& tracked_feature : pair.second.second)
				{
					if (tracked_feature->m_bDead) continue;
					int nObs = tracked_feature->m_vVisualObs.size();
					auto& lastOb = tracked_feature->m_vVisualObs[nObs - 1];
					ray1.push_back(lastOb.m_Ray);
					p1.push_back(lastOb.m_px);
					auto& lastSecondOb = tracked_feature->m_vVisualObs[nObs - 2];
					ray0.push_back(lastSecondOb.m_Ray);
					p0.push_back(lastSecondOb.m_px);
					goodTracks.push_back(tracked_feature.get());
				}

				std::vector<bool> vInliers;
				m_pTwoPointRansac->findInliers(p0,ray0, p1,ray1, pair.second.first, vInliers);
				for (int i = 0, n = vInliers.size(); i < n; ++i)
				{
					if (!vInliers[i])
					{
						auto& track = goodTracks[i];

						track->popObservation();
						track->m_bDead = true;
					}
				}
			}

#else
            for (const auto& tracked_feature : vTrackedFeatures)
            {
                if (tracked_feature->m_bDead) continue;
                int nObs = tracked_feature->m_vVisualObs.size();
                auto& lastOb = tracked_feature->m_vVisualObs[nObs - 1];
                ray1.push_back(lastOb.m_Ray);
                p1.push_back(lastOb.m_px);
                auto& lastSecondOb = tracked_feature->m_vVisualObs[nObs - 2];
                ray0.push_back(lastSecondOb.m_Ray);
                p0.push_back(lastSecondOb.m_px);
                goodTracks.push_back(tracked_feature.get());
            }

            std::vector<bool> vInliers;
            m_pTwoPointRansac->findInliers(p0,ray0,p1, ray1, dR, vInliers);
            for (int i = 0, n = vInliers.size(); i < n; ++i)
            {
                if (!vInliers[i])
                {
                    auto& track = goodTracks[i];

                    track->m_iDeadFrameId = track->m_vVisualObs.back().m_linkFrame->state->m_id;
                    track->popObservation();
                    track->m_bDead = true;
                    continue;
                }
                nGoodPoints++;
            }
#endif
            return nGoodPoints;
        }



        void _addBufferPoints(std::vector<std::shared_ptr<TrackedFeature>>& vDeadFeature)
        {
            constexpr int MAX_BUFFER_OBS = 5;
            for (auto trackedFeature : m_vTrackedFeatureNextUpdate)
            {
#if USE_KEYFRAME
            	if(trackedFeature->m_pState)
					assert(!trackedFeature->m_pState->bSlamPoint);
#endif
            	if (trackedFeature->m_vVisualObs.size() > MAX_BUFFER_OBS)
                    vDeadFeature.push_back(trackedFeature);
                else
                    trackedFeature->removeLinksInCamStates();
            }
#if OUTPUT_DEBUG_INFO
            printf("  Add Buffer Points:%d/%d\n", vDeadFeature.size(), m_vTrackedFeatureNextUpdate.size());
#endif
            m_vTrackedFeatureNextUpdate.clear();

        }

        void _addDeadPoints(std::list<TrackedFeaturePtr>& vTrackedFeatures,std::vector<std::shared_ptr<TrackedFeature>>& vDeadFeature)
        {
            constexpr int MIN_OBS = 4;
            int nDeadPoints2Updates = 0;
            int nDeadPointsAbandoned = 0;
            int nAlivePoints2Updates = 0;
#if USE_KEYFRAME
            constexpr int MIN_OBS_ALIVE = 6;
            constexpr int MIN_OBS_TRACKED = 6;

#else
            constexpr int MIN_OBS_ALIVE = 8;
            constexpr int MIN_OBS_TRACKED = 10;
#endif
            for (auto iter = vTrackedFeatures.begin(); iter != vTrackedFeatures.end();) {
                auto tracked_feature = *iter;

#if USE_KEYFRAME
                if (tracked_feature->m_pState && tracked_feature->m_pState->bSlamPoint) {
                    ++iter;
                	continue;
                }
#endif

            	
                if (tracked_feature->m_bDead) {
                    if (tracked_feature->m_nObs >= MIN_OBS_TRACKED && tracked_feature->m_vVisualObs.size()>=MIN_OBS) {
                        vDeadFeature.push_back(tracked_feature);
                        nDeadPoints2Updates++;
                    }
                    else {
                        tracked_feature->removeLinksInCamStates();
                        nDeadPointsAbandoned++;
                    }
                    iter = vTrackedFeatures.erase(iter);
                    continue;
                }
#if USE_KEYFRAME
                if (tracked_feature->m_vVisualObs.size() >= MIN_OBS_ALIVE&&tracked_feature->m_nObs>MIN_OBS_TRACKED) {
#else
                if (tracked_feature->m_vVisualObs.size() >= MIN_OBS_ALIVE) {
#endif
                    nAlivePoints2Updates++;
                    vDeadFeature.push_back(tracked_feature);
                }
                ++iter;
            }

#if OUTPUT_DEBUG_INFO
            printf("  Dead Points: %d Good / %d Bad\n", nDeadPoints2Updates, nDeadPointsAbandoned);
			printf("  AlivePoints to Update:%d\n", nAlivePoints2Updates);
#endif
        }

        void _pushPoints2Grid(const std::vector<std::shared_ptr<TrackedFeature>>& vDeadFeature)
        {
            static std::vector<std::vector<TrackedFeaturePtr> > vvGrid44(4 * 4);
            static CamModel* camModel = CamModel::getCamModel();
            static const int STEPX = camModel->width() / 4;
            static const int STEPY = camModel->height() / 4;

            auto comparator_less = [](const TrackedFeaturePtr& a, const TrackedFeaturePtr& b)
            {
                return a->m_bDead == b->m_bDead ? a->m_rayAngle < b->m_rayAngle : a->m_bDead < b->m_bDead;
            };

            auto selectTop2 = [&](const std::vector<TrackedFeaturePtr>& src, std::vector<TrackedFeaturePtr>& dst)
            {
                TrackedFeaturePtr pFirst = nullptr, pSecond = nullptr;
                if (!src.empty()) {

                    for (auto tracked_feature : src) {
                        if (!pSecond || comparator_less(pSecond, tracked_feature)) {
                            if (pSecond && pSecond->m_bDead)
                                m_vTrackedFeatureNextUpdate.push_back(pSecond);
                            pSecond = tracked_feature;
                            if (!pFirst || comparator_less(pFirst, pSecond)) {
                                std::swap(pFirst, pSecond);
                            }
                        }
                        else {
                            if (tracked_feature->m_bDead)
                                m_vTrackedFeatureNextUpdate.push_back(tracked_feature);
                        }
                    }
                    if (pSecond)
                        dst.push_back(pSecond);
                    if (pFirst && pFirst != pSecond)
                        dst.push_back(pFirst);
                }
            };


            for (auto deadFeature : vDeadFeature) {
                auto& ob = deadFeature->m_vVisualObs.back();
                vvGrid44[int(ob.m_px.x() / STEPX) + 4 * int(ob.m_px.y() / STEPY)].push_back(deadFeature);
            }
#if OUTPUT_DEBUG_INFO
            printf("# 4*4 Dead Points:\n");
			printf("____________________\n");
			printf("|%03d|%03d|%03d|%03d|\n", vvGrid44[0].size(), vvGrid44[1].size(), vvGrid44[2].size(), vvGrid44[3].size());
			printf("|%03d|%03d|%03d|%03d|\n", vvGrid44[4].size(), vvGrid44[5].size(), vvGrid44[6].size(), vvGrid44[7].size());
			printf("|%03d|%03d|%03d|%03d|\n", vvGrid44[8].size(), vvGrid44[9].size(), vvGrid44[10].size(), vvGrid44[11].size());
			printf("|%03d|%03d|%03d|%03d|\n", vvGrid44[12].size(), vvGrid44[13].size(), vvGrid44[14].size(), vvGrid44[15].size());
			printf("____________________\n");
#endif

            static int LUT[16] =
                    { 0,0,1,1,
                      0,0,1,1,
                      2,2,3,3,
                      2,2,3,3 };

            for (int i = 0; i < 16; ++i) {
                selectTop2(vvGrid44[i], m_Grid22[LUT[i]]);
            }

            for (auto& grid : m_Grid22) {
                std::sort(grid.begin(), grid.end(), comparator_less);
            }

            for (auto& grid : vvGrid44)
                grid.clear();

#if OUTPUT_DEBUG_INFO
            printf("# 2*2 Dead Points:\n");
			printf("____________________\n");
			printf("|  %03d  |  %03d  |\n", m_Grid22[0].size(), m_Grid22[1].size());
			printf("|  %03d  |  %03d  |\n", m_Grid22[2].size(), m_Grid22[3].size());
			printf("____________________\n");
#endif
        }

#if 1
        void _tryAddMsckfPoseConstraint(const std::list<TrackedFeaturePtr>&lTrackFeatures)
        {

        	
            int nPointsPerGrid = MAX_MSCKF_FEATURE_UPDATE_PER_FRAME / 4;
            int nPointsLeft = MAX_MSCKF_FEATURE_UPDATE_PER_FRAME;
            int nPointsAllAdded = 0;
            int nPointsTriangleFailed = 0;
            int nPointsMahalaFailed = 0;


#if USE_KEYFRAME
            int halfX = CamModel::getCamModel()->width() / 2;
            int halfY = CamModel::getCamModel()->height() / 2;
            int nPointsSlamPerGrid = MAX_POINT_SIZE / 4;
            std::vector<int> vPointsSLAMLeft{ 0,0, 0, 0 };
            std::vector<int> vPointsSLAMNow{ 0,0, 0, 0 };
            static std::vector<std::vector<TrackedFeature*>> m_slamPointGrid22(4);
            int nSlamPoint = 0;

            std::for_each(m_slamPointGrid22.begin(), m_slamPointGrid22.end(), []( auto& a) {a.clear(); });
            for (auto& point : lTrackFeatures)
            {
                if (point->m_pState && point->m_pState->bSlamPoint&& !point->m_bDead)
                {
                    float x = point->m_vVisualObs.back().m_px.x();
                    float y = point->m_vVisualObs.back().m_px.y();
                    if (x < halfX)
                        if (y < halfY) {
                            vPointsSLAMNow[0]++;
                            m_slamPointGrid22[0].push_back(point.get());
                        }
                        else {
                            vPointsSLAMNow[2]++;
                            m_slamPointGrid22[2].push_back(point.get());
                        }
                    else if (y < halfY) {
							
                    	vPointsSLAMNow[1]++;
                        m_slamPointGrid22[1].push_back(point.get());
                    }
                    else {
                        m_slamPointGrid22[3].push_back(point.get());
                    	vPointsSLAMNow[3]++;
                    }
                    nSlamPoint++;
                }
            	
            }
            if (nSlamPoint<16)
            {
                for (int i = nSlamPoint; i < 16; i++)
                {
                    int k = 0;
                    for (int j=0;j<3;j++)
                    {
                        k = vPointsSLAMNow[k] <= vPointsSLAMNow[j + 1] ? k : j + 1;
                    }
                    vPointsSLAMLeft[k]++;
                    vPointsSLAMNow[k] ++;
                }
            }else
            {
	            for (int i=0;i<4;++i)
                {
		            if(vPointsSLAMNow[i]>4)
		            {
                        m_slamPointGrid22[i][2]->m_pState->m_bNextMargin = true;
                        vPointsSLAMLeft[i]++;
		            }
	            }
            }
        	
            LOGI("SlamCnt:%d %d %d %d %d", nSlamPoint, vPointsSLAMLeft[0], vPointsSLAMLeft[1], vPointsSLAMLeft[2], vPointsSLAMLeft[3]);
            nPointsLeft = (MAX_OBS_SIZE - MAX_ADDITIONAL_POINT * MAX_WINDOW_SIZE * 2 - nSlamPoint * 5) / (MAX_WINDOW_SIZE * 2);
            nPointsPerGrid = nPointsLeft / 4;

#endif

            std::vector<int> vPointsLeft = { nPointsPerGrid,nPointsPerGrid,nPointsPerGrid,nPointsPerGrid };


#if USE_KEYFRAME

#endif
        	
            auto triangleAndVerify = [&](TrackedFeaturePtr& track)
            {
                if (track->triangulate())
                {
#if OUTPUT_DEBUG_INFO
                    printf("#### Triangulation Success\n");
#endif
                    if (m_pSquareRootSolver->computeJacobians(track.get())) {

                        if (m_pSquareRootSolver->MahalanobisTest(track->m_pState)) {
                            nPointsAllAdded++;
                            return true;
                        }
                        nPointsMahalaFailed++;
                        return false;
                    }
                    return false;
                }
                nPointsTriangleFailed++;
#if OUTPUT_DEBUG_INFO
                printf("#### Triangulation Fail\n");
#endif
                return false;
            };

        	
            auto selectPoints = [&]()
            {
#if OUTPUT_DEBUG_INFO
                int ii = 0;
                int jInGrid = 0;

#endif
                for (int i = 0;i<4;++i)
                {
                    auto& grid = m_Grid22[i];
#if OUTPUT_DEBUG_INFO

                    printf("## Grid %d in 2*2:\n", ii);
                    ii++;
#endif
                    while (vPointsLeft[i] && !grid.empty())
                    {
#if OUTPUT_DEBUG_INFO
                        printf("### %d point in Grid %d\n", jInGrid++, ii);
#endif
                        auto& ft = grid.back();
                        if (triangleAndVerify(ft))
                        {
                           
                            --nPointsLeft;
#if OUTPUT_DEBUG_INFO
                            ++nPointsAllAdded;
#endif
#if USE_KEYFRAME
                            if (!ft->m_bDead && ft->m_bSlamPointCandidate && vPointsSLAMLeft[i]) {//If it is a slam point

                                m_pSquareRootSolver->addSlamPoint(ft->m_pState);
                                vPointsSLAMLeft[i]--;
                            }
                            else{
#endif
                                m_pSquareRootSolver->addMsckfPoint(ft->m_pState);
                                ft->m_bDead = true;
#if USE_KEYFRAME
                            }
#endif
                            vPointsLeft[i]--;

                        	m_vTrackedFeatureToUpdate.push_back(ft);


                        }
                        else
                            ft->m_bDead = false;
                        grid.pop_back();
                    }


                }

            };
            auto bufferPoints = [&]()
            {
                for (auto& grid : m_Grid22)
                {
                    for (auto& ft : grid)
                    {
                        if (ft->m_bDead)
                        {
                            m_vTrackedFeatureNextUpdate.push_back(ft);
                        }
                    }
                    grid.clear();
                }
            };

            selectPoints();

            if (nPointsLeft) {

                int nMoreGrid = 0;
            	for (auto&ptLeft:vPointsLeft)
            	{
                    if (ptLeft == 0)
                        nMoreGrid++;
            	}
            	for (auto&ptLeft:vPointsLeft)
            	{
                    if (ptLeft == 0)
                        ptLeft = nPointsLeft / nMoreGrid;
                    else
                        ptLeft = 0;
            	}
                selectPoints();
            }
#if OUTPUT_DEBUG_INFO
            printf("### %d Points to Update\n", nPointsAllAdded);
#endif
            printf("### nPointsAdded:%d nPointsTriangleFailed:%d nPointsMahalaFailed:%d\n", nPointsAllAdded, nPointsTriangleFailed, nPointsMahalaFailed);
            bufferPoints();
        	
        }

#endif


#if 0
        void _tryAddMsckfPoseConstraint()
        {
            int nPointsPerGrid = MAX_MSCKF_FEATURE_UPDATE_PER_FRAME / 4;
            int nPointsLeft = MAX_MSCKF_FEATURE_UPDATE_PER_FRAME;
            int nPointsAllAdded = 0;
            int nPointsTriangleFailed = 0;
            int nPointsMahalaFailed = 0;

            auto triangleAndVerify = [&](TrackedFeaturePtr& track)
            {
                if (track->triangulate())
                {
#if OUTPUT_DEBUG_INFO
                    printf("#### Triangulation Success\n");
#endif
                    if (m_pSquareRootSolver->computeJacobians(track)) {

                        if (m_pSquareRootSolver->MahalanobisTest(track->m_pState)) {
                            nPointsAllAdded++;
                        	return true;
                        }
                        nPointsMahalaFailed++;
                        return false;
                    }
                    return false;
                }
                nPointsTriangleFailed++;
#if OUTPUT_DEBUG_INFO
                printf("#### Triangulation Fail\n");
#endif
                return false;
            };
            auto selectPoints = [&]()
            {
#if OUTPUT_DEBUG_INFO
                int ii = 0;
				int jInGrid = 0;

#endif
                for (auto& grid : m_Grid22)
                {
#if OUTPUT_DEBUG_INFO

                    printf("## Grid %d in 2*2:\n", ii);
					ii++;
#endif
                    int nPointsAdded = 0;
                    while (nPointsAdded < nPointsPerGrid && !grid.empty())
                    {
#if OUTPUT_DEBUG_INFO
                        printf("### %d point in Grid %d\n", jInGrid++, ii);
#endif
                        auto& ft = grid.back();
                        if (triangleAndVerify(ft))
                        {
                            ++nPointsAdded;
                            --nPointsLeft;
#if OUTPUT_DEBUG_INFO
                            ++nPointsAllAdded;
#endif
                            m_pSquareRootSolver->addMsckfPoint(ft->m_pState);

                            m_vTrackedFeatureToUpdate.push_back(ft);

                            ft->m_bDead = true;

                        }
                        else
                            ft->m_bDead = false;
                        grid.pop_back();
                    }


                }

            };
            auto bufferPoints = [&]()
            {
                for (auto& grid : m_Grid22)
                {
                    for (auto& ft : grid)
                    {
                        if (ft->m_bDead)
                        {
                            m_vTrackedFeatureNextUpdate.push_back(ft);
                        }
                    }
                    grid.clear();
                }
            };

            selectPoints();
            nPointsPerGrid = nPointsLeft / 4;

            if (nPointsPerGrid) {
                selectPoints();
            }
#if OUTPUT_DEBUG_INFO
            printf("### %d Points to Update\n", nPointsAllAdded);
#endif
            printf("### nPointsAdded:%d nPointsTriangleFailed:%d nPointsMahalaFailed:%d\n", nPointsAllAdded, nPointsTriangleFailed, nPointsMahalaFailed);
            bufferPoints();
        }
#endif

        void doDataAssociation(std::list<TrackedFeaturePtr>& vTrackedFeatures,
            bool bStatic)
        {
            m_vTrackedFeatureToUpdate.clear();

#if USE_STATIC_DETECTION
        	if(bStatic)
        	{
                m_vTrackedFeatureNextUpdate.clear();
                return;
        	}

#endif
            static std::vector<TrackedFeaturePtr> vDeadFeature;

            _addBufferPoints(vDeadFeature);

            _addDeadPoints(vTrackedFeatures,vDeadFeature);

            _pushPoints2Grid(vDeadFeature);

        	vDeadFeature.clear();

            _tryAddMsckfPoseConstraint(vTrackedFeatures);
        }


    }
}
