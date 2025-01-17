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
    void AddFrameAdapter(FrameAdapter* adapter) { frame_adapter_ = adapter; }
    void AddWorldPointAdapter(WorldPointAdapter* adapter) {
        world_point_adapter_ = adapter;
    }
    void OnImageReceived(const ImageData::Ptr imageData) override;

   private:
    void DoWhatYouNeedToDo() override;

    bool HaveThingsTodo() override;

    std::string dat_dir_;
    FILE* imu_file_ = nullptr;
    FILE* cam_file_ = nullptr;

    std::thread* thread_ = nullptr;
    FrameAdapter* frame_adapter_ = nullptr;
    WorldPointAdapter* world_point_adapter_ = nullptr;
    ImageData::Ptr image_data_;
    std::atomic<bool> flag_have_image_;
    std::mutex image_mutex_;
};

}  // namespace DeltaVins