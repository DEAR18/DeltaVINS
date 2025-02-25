#include "IO/dataBuffer/GnssBuffer.h"

namespace DeltaVins {

void GnssBuffer::OnNavSatFixReceived(const NavSatFixData& navSatFixData) {
    getHeadNode() = navSatFixData;
    PushIndex();
}

}  // namespace DeltaVins
