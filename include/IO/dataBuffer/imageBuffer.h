#pragma once
#include "IO/dataSource/dataSource.h"
#include "dataStructure/ringBuffer.h"

namespace DeltaVins {

class ImageBuffer : public CircularBuffer<ImageData::Ptr, 6> {
   public:
    friend class DataRecorder;
    static ImageBuffer& getInstance() {
        static ImageBuffer imageBuffer;
        return imageBuffer;
    }
    void pushImage(const ImageData::Ptr imageData) {
        _buf[m_head] = imageData;
        pushIndex();
        if (full()) {
            LOGW("Image Buffer is Full");
        }
    }

    ImageData::Ptr popHeadImage() { return _buf[getDeltaIndex(m_head, -1)]; };
    ImageData::Ptr popTailImage() {
        if (empty()) return nullptr;
        int tail = m_tail;
        popIndex();
        return _buf[tail];
    }
    ~ImageBuffer() {};

   private:
    ImageBuffer() {};
};

}  // namespace DeltaVins