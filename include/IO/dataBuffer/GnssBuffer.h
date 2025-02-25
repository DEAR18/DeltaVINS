#pragma once
#include "dataStructure/sensorStructure.h"
#include "dataStructure/ringBuffer.h"
#include "IO/dataSource/dataSource.h"

namespace DeltaVins {
class GnssBuffer : public CircularBuffer<NavSatFixData, 10>,
                   public DataSource::NavSatFixObserver {
   public:
    friend class DataRecorder;
    static GnssBuffer& Instance() {
        static GnssBuffer gnssBuffer;
        return gnssBuffer;
    }
    ~GnssBuffer() = default;

    void OnNavSatFixReceived(const NavSatFixData& navSatFixData) override;

   private:
    GnssBuffer() = default;
};
}  // namespace DeltaVins