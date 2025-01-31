#pragma once

#include "IO/dataSource/dataSource.h"
#include "dataStructure/ringBuffer.h"

namespace DeltaVins {
struct ImuPreintergration;

class ImuBuffer : public CircularBuffer<ImuData, 10>,
                  public DataSource::ImuObserver {
   public:
    friend class DataRecorder;
    static ImuBuffer& Instance() {
        static ImuBuffer imuBuffer;
        return imuBuffer;
    }
    void OnImuReceived(const ImuData& imuData) override;
    Vector3f GetGravity(long long timestamp);
    Vector3f GetGravity();

    long long GetNextSyncTimestamp(int& imuIdx, long long lastTimeStamp);
    bool GetDataByBinarySearch(ImuData& imuData) const;

    bool ImuPreIntegration(ImuPreintergration& ImuTerm) const;

    void UpdateBias(const Vector3f& dBg, const Vector3f& dBa);

    void SetBias(const Vector3f& bg, const Vector3f& ba);

    void SetZeroBias();

    void GetBias(Vector3f& bg, Vector3f& ba) const;

    bool DetectStatic(long long timestamp) const;

   private:
    ImuBuffer();

    MatrixXf noise_cov_;

    Vector3f gyro_bias_;
    Vector3f acc_bias_;

    std::mutex gravity_mutex_;
    Vector3f gravity_;
};

}  // namespace DeltaVins