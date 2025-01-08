//
// Created by chenguojun on 2020/3/19.
//
#pragma once
#include <FrameAdapter.h>
#include <IO/dataSource/dataSource.h>
#include <WorldPointAdapter.h>
namespace DeltaVins {

class DataRecorder : public DataSource::ImageObserver, public AbstractModule {
   public:
    DataRecorder();
    ~DataRecorder();
    using Ptr = std::shared_ptr<DataRecorder>;
    void addFrameAdapter(FrameAdapter* adapter) { m_frameAdapter = adapter; }
    void addWorldPointAdapter(WorldPointAdapter* adapter) {
        m_worldPointAdapter = adapter;
    }
    void onImageReceived(const ImageData::Ptr imageData) override;

   private:
    void doWhatYouNeedToDo() override;

    bool haveThingsTodo() override;

    std::string m_datDir;
    FILE* m_imuFile = nullptr;
    FILE* m_camFile = nullptr;

    std::thread* m_thread = nullptr;
    FrameAdapter* m_frameAdapter = nullptr;
    WorldPointAdapter* m_worldPointAdapter = nullptr;
    ImageData::Ptr imageData;
    std::atomic<bool> haveImage;
    std::mutex imageMutex;
};

}  // namespace DeltaVins