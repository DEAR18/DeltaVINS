#pragma once

#include <Eigen/Core>
#include <vector>

struct WorldPointGL {
    WorldPointGL() {};
    WorldPointGL(const Eigen::Vector3f& p, int id) : m_id(id) { m_P = p; }

    int32_t m_id;
    Eigen::Vector3f m_P;
};

struct WorldPointAdapter {
    virtual ~WorldPointAdapter() = default;
    virtual void pushWorldPoint(const std::vector<WorldPointGL>& v_Point3f) {};
};
