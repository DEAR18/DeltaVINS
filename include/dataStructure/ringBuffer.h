#pragma once

#include <utils/log.h>

#include <cassert>
#include <cstring>
#include <typeinfo>

namespace DeltaVins {

template <typename T, int N>
struct CircularBuffer {
   public:
    using BufferIndex = int;
    CircularBuffer() : _BUFSIZE(1 << N), _END(_BUFSIZE - 1) {
        buf_ = new T[_BUFSIZE];
    }

    ~CircularBuffer() {
        if (buf_) delete[] buf_;
    }

    bool empty() const { return head_ == tail_; };

    bool Full() const { return ((head_ + 1) & _END) == tail_; };

    BufferIndex getDeltaIndex(BufferIndex index, BufferIndex delta) const {
        return (index + _BUFSIZE + delta) & _END;
    }

    T& getHeadNode() { return buf_[head_]; };

    //
    void PushIndex() {
        if (getDeltaIndex(head_, 1) == tail_) {
            tail_ = tail_ + 1 & _END;
        }
        head_ = head_ + 1 & _END;
    }

    //
    void PopIndex() { tail_ = (tail_ + 1) & _END; }

    enum BinarySearchResultType { Left = 0, Right = 1, Exact = -1 };

    template <typename Key>
    BufferIndex binarySearch(const Key& key,
                             BinarySearchResultType leftOrRight) const {
        // If buffer is empty,return false
        if (empty()) return -3;
        int head_M1 = getDeltaIndex(head_, -1);
        int tail_P1 = getDeltaIndex(tail_, 5);

        if (buf_[head_M1] < key) {
            // if constexpr (typeid(key) == typeid(long long int))
            // LOGW("Image is Faster than IMU,tail:%d %d head:%d
            // %dkey:%lld,tail_P1:%lld,head_M1:%lld",tail_,tail_P1,head_,head_M1,key,buf_[tail_P1].timestamp,buf_[head_M1].timestamp)
            return -1;
        }
        if (buf_[tail_P1] > key) {
            // if constexpr (typeid(key) == typeid(int))
            // LOGE("Image Idx:%d tailIdx:%d",key,buf_[tail_P1].idx);
            return -3;
        }

        int left;
        int right;
        // set binary search left and right side
        if (head_M1 > tail_P1) {
            left = tail_P1;
            right = head_M1;
        } else {
            if (buf_[0] <= key) {
                left = 0;
                right = head_M1;
            } else {
                if (buf_[_END] <= key) {
                    left = _END;
                    right = 0;
                } else {
                    left = tail_P1;
                    right = _END;
                }
            }
        }
        // do binary search
        while (left + 1 < right) {
            int mid = (left + right) / 2;
            if (buf_[mid] == key) return mid;
            if (buf_[mid] <= key)
                left = mid;
            else
                right = mid;
        }
        if (leftOrRight == 0)
            return left;
        else if (leftOrRight == 1)
            return right;
        else if (leftOrRight == -1)
            return -2;
        assert(0 && "Wrong LeftOrRight");
        return -3;
    }

   protected:
    const BufferIndex _BUFSIZE;
    const BufferIndex _END;
    T* buf_ = nullptr;
    // std::atomic<BufferIndex> head_, tail_;
    BufferIndex head_, tail_;
};

}  // namespace DeltaVins