#pragma once
#include "IO/dataSource/dataSource.h"
#include "dataStructure/ringBuffer.h"

namespace DeltaVins {
struct OdomPreintergration;

class OdometerBuffer : public CircularBuffer<OdometerData, 10>,
                       public DataSource::OdometerObserver {
   public:
    friend class DataRecorder;
    static OdometerBuffer& Instance() {
        static OdometerBuffer odom_buffer;
        return odom_buffer;
    }
    ~OdometerBuffer() = default;

    void OnOdometerReceived(const OdometerData& odometerData) override;

    // bool OdomPreIntegration(int64_t start_t, int64_t end_t,
    //                         OdomPreintergration* odom_term) const;

    bool OdomVelocity(int64_t query_time, float* velocity) const;

   private:
    OdometerBuffer();

    Matrix2f odom_noise_cov_{
        Matrix2f::Zero()};  // order: angular_velocity, velocity
};

}  // namespace DeltaVins
