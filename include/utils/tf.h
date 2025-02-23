#pragma once

#include <Eigen/Dense>
#include <set>
#include <unordered_map>
namespace DeltaVins {

/**
 * @brief Transform from parent frame to child frame
 */
template <typename T>
struct Transform {
    long long timestamp;  // if timestamp is 0, it means the transform is static
    std::string frame_id;        // frame_id of the parent
    std::string child_frame_id;  // frame_id of the child
    // P_parent = T_parent_child * P_child
    Eigen::Transform<T, 3, Eigen::Isometry>
        T_parent_child;  // transform child frame with respect to parent frame

    Transform() = default;
    Transform(long long timestamp, const std::string& frame_id,
              const std::string& child_frame_id,
              const Eigen::Transform<T, 3, Eigen::Isometry>& T_parent_child)
        : timestamp(timestamp),
          frame_id(frame_id),
          child_frame_id(child_frame_id),
          T_parent_child(T_parent_child) {}

    static Transform<T> CreateFromRotationAndPosition(
        long long timestamp, const std::string& frame_id,
        const std::string& child_frame_id,
        const Eigen::Matrix<T, 3, 3>& rotation,
        const Eigen::Matrix<T, 3, 1>& position) {
        Eigen::Isometry3f T_parent_child;
        T_parent_child.linear() = rotation;
        T_parent_child.translation() = -rotation.transpose() * position;
        return Transform<T>(timestamp, frame_id, child_frame_id,
                            T_parent_child);
    }
    Eigen::Matrix<T, 3, 1> Translation() const {
        return T_parent_child.translation();
    }

    Eigen::Matrix<T, 3, 3> Rotation() const {
        return T_parent_child.rotation().matrix();
    }

    Eigen::Matrix<T, 3, 3> Position() const {
        return -T_parent_child.rotation().matrix() *
               T_parent_child.translation();
    }

    Eigen::Matrix<T, 3, 1> TransformPoint(
        const Eigen::Matrix<T, 3, 1>& point) const {
        return T_parent_child.linear() * point + T_parent_child.translation();
    }

    Transform<T> Inverse() const {
        return Transform<T>(timestamp, child_frame_id, frame_id,
                            T_parent_child.inverse());
    }

    Transform<T> operator*(const Transform<T>& other) const {
#ifndef NDEBUG
        if (other.frame_id != child_frame_id) {
            LOGW("Transform::operator* : other.frame_id != child_frame_id");
        }
#endif
        return Transform<T>(timestamp, frame_id, other.child_frame_id,
                            T_parent_child * other.T_parent_child);
    }
};

/**
 * @brief A simple transform manager for multiple frames
 * @tparam T floating point type
 * @note  All tramsforms should be added to the manager before use
 */

template <typename T>
class Tfs {
   public:
    static Tfs& Instance() {
        static Tfs instance;
        return instance;
    }

    void AddStaticTransform(const Transform<T>& tf) {
        // Interative Build Transform Graph
        for (const auto& [frame_id, tf2] : tf_chains_[tf.frame_id]) {
            // tf2 is the transform from tf.frame_id to frame_id
            // we need to find the transform from tf.child_frame_id to frame_id
            // the transform is tf_chains_[tf.child_frame_id][frame_id]
            // we need to update tf_chains_[tf.child_frame_id][frame_id]
            tf_chains_[tf.child_frame_id][frame_id] = tf2 * tf;
            tf_chains_[frame_id][tf.child_frame_id] =
                tf.Inverse() * tf2.Inverse();
        }
        for (const auto& [frame_id, tf2] : tf_chains_[tf.child_frame_id]) {
            // tf2 is the transform from tf.child_frame_id to frame_id
            // we need to find the transform from tf.frame_id to frame_id
            // the transform is tf_chains_[tf.frame_id][frame_id]
            // we need to update tf_chains_[tf.frame_id][frame_id]
            tf_chains_[tf.frame_id][frame_id] = tf2 * tf.Inverse();
            tf_chains_[frame_id][tf.frame_id] = tf * tf2.Inverse();
        }

        frame_ids_.insert(tf.frame_id);
        frame_ids_.insert(tf.child_frame_id);
        tf_chains_[tf.child_frame_id][tf.frame_id] = tf;
        tf_chains_[tf.frame_id][tf.child_frame_id] = tf.Inverse();
    }

    bool GetTransform(const std::string& frame_id,
                      const std::string& child_frame_id,
                      Transform<T>& tf) const {
        if (tf_chains_.find(child_frame_id) == tf_chains_.end() ||
            tf_chains_.at(child_frame_id).find(frame_id) ==
                tf_chains_.at(child_frame_id).end()) {
            return false;
        }
        tf = tf_chains_.at(child_frame_id).at(frame_id);
        return true;
    }

    std::vector<std::string> GetAllFrameIds() const {
        return std::vector<std::string>(frame_ids_.begin(), frame_ids_.end());
    }

   private:
    Tfs() = default;
    std::set<std::string> frame_ids_;

    std::unordered_map<std::string,
                       std::unordered_map<std::string, Transform<T>>>
        tf_chains_;
};

}  // namespace DeltaVins