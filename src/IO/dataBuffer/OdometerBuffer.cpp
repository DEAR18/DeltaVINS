#include "IO/dataBuffer/OdometerBuffer.h"

#include "Algorithm/Odometer/OdomPreintergration.h"
#include "utils/utils.h"

namespace DeltaVins {

OdometerBuffer::OdometerBuffer() {
    float angular_velocity_noise = 0.05f;
    float velocity_noise = 0.5f;
    odom_noise_cov_(0, 0) = angular_velocity_noise * angular_velocity_noise;
    odom_noise_cov_(1, 1) = velocity_noise * velocity_noise;
}

void OdometerBuffer::OnOdometerReceived(const OdometerData& odometerData) {
    getHeadNode() = odometerData;
    PushIndex();
}

/*
bool OdometerBuffer::OdomPreIntegration(int64_t start_t, int64_t end_t,
                                        OdomPreintergration* odom_term) const {
    if (end_t <= start_t) {
        LOGE("input timestamp disorder, start time:%lld end time:%lld",
             (long long)start_t, (long long)end_t);
        return false;
    }
    int index0 = binarySearch<long long>(start_t, Left);
    if (index0 < 0) {
        LOGE("start time querying odometer buffer failed, start time:%lld",
             (long long)start_t);
        return false;
    }
    int index1 = binarySearch<long long>(end_t, Left);
    for (int i = 0; i < 5 && index1 < 0; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        index1 = binarySearch<long long>(end_t, Left);
        LOGI("waiting for odom, %d", i);
    }
    if (index1 < 0) {
        LOGE("end time querying odometer buffer failed, end time:%lld",
             (long long)end_t);
        return false;
    }
    if (index0 == index1) {
        LOGW("no enough odometer data for preintegration.");
        return false;
    }

    odom_term->t0 = start_t;
    odom_term->t1 = end_t;

    int index = index0;
    int index_end = getDeltaIndex(index1, 1);
    float dt = 0.f;
    int cnt = 0;
    while (index != index_end) {
        float velocity = 0.f;
        float angular_velocity = 0.f;

        auto& odom_data = buf_[index];
        int nextIndex = getDeltaIndex(index, 1);
        auto& next_odom_data = buf_[nextIndex];
        if (index == index0) {
            dt = (next_odom_data.timestamp - odom_term->t0) * 1e-9;
            float a = next_odom_data.timestamp - odom_term->t0;
            float b = next_odom_data.timestamp - odom_data.timestamp;
            float k = a / b;
            velocity = linearInterpolate(odom_data.velocity,
                                         next_odom_data.velocity, 1 - k);
            angular_velocity =
                linearInterpolate(odom_data.angularVelocity,
                                  next_odom_data.angularVelocity, 1 - k);
        } else if (index == index1) {
            dt = (odom_term->t1 - odom_data.timestamp) * 1e-9;
            float a = next_odom_data.timestamp - odom_term->t1;
            float b = next_odom_data.timestamp - odom_data.timestamp;
            float k = a / b;
            velocity = linearInterpolate(odom_data.velocity,
                                         next_odom_data.velocity, k);
            angular_velocity = linearInterpolate(
                odom_data.angularVelocity, next_odom_data.angularVelocity, k);
        } else {
            dt = (next_odom_data.timestamp - odom_data.timestamp) * 1e-9;
            velocity = (odom_data.velocity + next_odom_data.velocity) * 0.5f;
            angular_velocity =
                (odom_data.angularVelocity + next_odom_data.angularVelocity) *
                0.5f;
        }

        Matrix3f F = Matrix3f::Identity();
        F(1, 0) = -velocity * dt * std::sin(odom_term->dtheta);
        F(2, 0) = velocity * dt * std::cos(odom_term->dtheta);

        MatrixXf G = MatrixXf::Zero(3, 2);
        G(0, 0) = dt * dt;
        G(1, 1) = dt * std::cos(odom_term->dtheta);
        G(2, 1) = dt * std::sin(odom_term->dtheta);

        odom_term->cov = F * odom_term->cov * F.transpose() +
                         G * odom_noise_cov_ * G.transpose();

        odom_term->dtheta += angular_velocity * dt;
        odom_term->dx += velocity * dt * std::cos(odom_term->dtheta);
        odom_term->dy += velocity * dt * std::sin(odom_term->dtheta);

        index = nextIndex;
        ++cnt;
    }

    odom_term->dT += odom_term->t1 - odom_term->t0;

    return true;
}
*/

bool OdometerBuffer::OdomVelocity(int64_t query_time, float* velocity) const {
    int index_left = binarySearch<long long>(query_time, Left);
    if (index_left < 0) {
        LOGE("time querying odometer buffer failed, time:%lld",
             (long long)query_time);
        return false;
    }
    const auto& odom_data_left = buf_[index_left];

    int index_right = getDeltaIndex(index_left, 1);
    if (index_right > 0) {
        const auto& odom_data_right = buf_[index_right];
        float a = query_time - odom_data_left.timestamp;
        float b = odom_data_right.timestamp - odom_data_left.timestamp;
        float k = a / b;
        *velocity = linearInterpolate(odom_data_left.velocity,
                                      odom_data_right.velocity, k);
    } else if ((query_time - odom_data_left.timestamp) * 1e-6 < 10.0) {
        *velocity = odom_data_left.velocity;
    } else {
        return false;
    }

    return true;
}

}  // namespace DeltaVins
