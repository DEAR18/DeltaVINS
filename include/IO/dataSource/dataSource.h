#pragma once
#include <opencv2/opencv.hpp>

#include "dataStructure/IO_Structures.h"
#include "framework/abstractModule.h"

namespace DeltaVins {

class DataSource : public AbstractModule {
   public:
    struct ImuObserver {
        virtual void OnImuReceived(const ImuData& imuData) = 0;
    };

    struct ImageObserver {
        virtual void OnImageReceived(const ImageData::Ptr imageData) = 0;
    };

    using Ptr = std::shared_ptr<DataSource>;

    struct _ImageData {
        long long timestamp;
        std::string imagePath;
    };

   public:
    DataSource() {};
    ~DataSource() {};

    void AddImuObserver(ImuObserver* observer) {
        std::lock_guard<std::mutex> lck(mtx_imu_observer_);
        imu_observers_.push_back(observer);
    }
    void AddImageObserver(ImageObserver* observer) {
        std::lock_guard<std::mutex> lck(mtx_image_observer_);
        image_observers_.push_back(observer);
    }
    void DeleteImuObserver(ImuObserver* observer) {
        std::lock_guard<std::mutex> lck(mtx_imu_observer_);
        auto it =
            std::find(imu_observers_.begin(), imu_observers_.end(), observer);
        if (it != imu_observers_.end()) imu_observers_.erase(it);
    }
    void DeleteImageObserver(ImageObserver* observer) {
        std::lock_guard<std::mutex> lck(mtx_image_observer_);
        auto it = std::find(image_observers_.begin(), image_observers_.end(),
                            observer);
        if (it != image_observers_.end()) image_observers_.erase(it);
    }

   protected:
    std::vector<ImuObserver*> imu_observers_;
    std::vector<ImageObserver*> image_observers_;
    std::mutex mtx_imu_observer_;
    std::mutex mtx_image_observer_;
};

}  // namespace DeltaVins