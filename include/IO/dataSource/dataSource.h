#pragma once
#include <opencv2/opencv.hpp>

#include "dataStructure/IO_Structures.h"
#include "framework/abstractModule.h"

namespace DeltaVins {

class DataSource : public AbstractModule {
   public:
    struct ImuObserver {
        virtual void onImuReceived(const ImuData& imuData) = 0;
    };

    struct ImageObserver {
        virtual void onImageReceived(const ImageData::Ptr imageData) = 0;
    };

    using Ptr = std::shared_ptr<DataSource>;

    struct _ImageData {
        long long timestamp;
        std::string imagePath;
    };

   public:
    DataSource() {};
    ~DataSource() {};

    void addImuObserver(ImuObserver* observer) {
        std::lock_guard<std::mutex> lck(m_mtx_ImuObserver);
        m_v_imuObservers.push_back(observer);
    }
    void addImageObserver(ImageObserver* observer) {
        std::lock_guard<std::mutex> lck(m_mtx_imageObserver);
        m_v_imageObservers.push_back(observer);
    }
    void deleteImuObserver(ImuObserver* observer) {
        std::lock_guard<std::mutex> lck(m_mtx_ImuObserver);
        auto it = std::find(m_v_imuObservers.begin(), m_v_imuObservers.end(),
                            observer);
        if (it != m_v_imuObservers.end()) m_v_imuObservers.erase(it);
    }
    void deleteImageObserver(ImageObserver* observer) {
        std::lock_guard<std::mutex> lck(m_mtx_imageObserver);
        auto it = std::find(m_v_imageObservers.begin(),
                            m_v_imageObservers.end(), observer);
        if (it != m_v_imageObservers.end()) m_v_imageObservers.erase(it);
    }

   protected:
    std::vector<ImuObserver*> m_v_imuObservers;
    std::vector<ImageObserver*> m_v_imageObservers;
    std::mutex m_mtx_ImuObserver;
    std::mutex m_mtx_imageObserver;
};

}  // namespace DeltaVins