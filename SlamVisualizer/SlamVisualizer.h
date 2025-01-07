#pragma once
#include <pangolin/pangolin.h>
#include <vector>
#include <Eigen/Dense>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <thread>


#include "FrameAdapter.h"
#include "WorldPointAdapter.h"

class SlamVisualizer :public FrameAdapter, public WorldPointAdapter {

public:
	typedef std::shared_ptr<SlamVisualizer> Ptr;
	void pushViewMatrix(std::vector<FrameGL>& v_Rwc) override;

	void pushImageTexture(unsigned char* imageTexture, const int width, const int height, const int channels) override;

	void pushWorldPoint(const std::vector<WorldPointGL>& v_Point3f) override;

	void finishFrame() override;


	SlamVisualizer(int width, int height);
	~SlamVisualizer();

	void start();

	void join();

	void stop();

	void detach();


private:

	void _drawWorldAxis();
	void _saveFrames();
	void render();
	void _drawImageTexture();
	void _drawGeometry();
	void _clear();
	void _drawWorldPoints();
	void _drawCameraFrustum();

	int m_width, m_height;
	int m_channels;

	unsigned char* m_pImageTexture = nullptr;
	pangolin::GlTexture m_GLTexture;


	std::vector<float> m_pWorldPoints;
	int	m_iPointBufferSize;
	int m_iPointSize;
	std::vector<FrameGL> m_vFrames;


	std::mutex m_mtxPoint;
	std::mutex m_mtxFrame;
	std::mutex m_mtxImageTexture;
	std::atomic_bool m_bFrameByFrame;
	std::mutex m_mtxFrameByFrame;
	std::condition_variable m_cvFrameByFrame;

	std::thread* m_thread = nullptr;


	pangolin::OpenGlRenderState m_viewMatrix;
	pangolin::OpenGlRenderState m_FollowMatrix;
	pangolin::OpenGlMatrix T_wc;
};
