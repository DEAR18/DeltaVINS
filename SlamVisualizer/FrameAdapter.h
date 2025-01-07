#pragma once

#include <vector>
#include <Eigen/Dense>

struct FrameGL
{
	FrameGL() {};
	FrameGL(const Eigen::Matrix3f& Rwc, const Eigen::Vector3f& t, int id) :m_id(id), m_type(1)
	{
		m_Twc.setIdentity();
		m_Twc.topLeftCorner<3, 3>() = Rwc;
		m_Twc.topRightCorner<3, 1>() = t;
	};
	Eigen::Matrix4f m_Twc;
	int32_t				m_id;
	int32_t			    m_type;
};



struct FrameAdapter
{
	virtual ~FrameAdapter() = default;
	virtual void pushViewMatrix(std::vector<FrameGL>& v_Rcw) {};
	virtual void pushImageTexture(unsigned char* imageTexture, const int width, const int height, const int channels) {};
	virtual void finishFrame() {};
};