#pragma once
#include "IO/dataSource/dataSource.h"
#include "dataStructure/ringBuffer.h"

namespace DeltaVins {

class ImageBuffer : public CircularBuffer<ImageData::Ptr, 6> {
   public:
    friend class DataRecorder;
    static ImageBuffer& Instance() {
        static ImageBuffer imageBuffer;
        return imageBuffer;
    }
    void PushImage(const ImageData::Ptr imageData) {
        buf_[head_] = imageData;
        PushIndex();
        if (Full()) {
            LOGW("Image Buffer is Full");
        }
    }

    ImageData::Ptr PopHeadImage() { return buf_[getDeltaIndex(head_, -1)]; };
    ImageData::Ptr PopTailImage() {
        if (empty()) return nullptr;
        int tail = tail_;
        PopIndex();
        return buf_[tail];
    }
    ~ImageBuffer() {};

   private:
    ImageBuffer() {};
};

}  // namespace DeltaVins