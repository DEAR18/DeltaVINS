#pragma once

#include <FrameAdapter.h>
#include <WorldPointAdapter.h>

#include "Algorithm/VIOAlgorithm.h"
#include "IO/dataSource/dataSource.h"
#include "abstractModule.h"
namespace DeltaVins {

class VIOModule : public AbstractModule, public DataSource::ImageObserver {
   public:
    using Ptr = std::shared_ptr<VIOModule>;
    struct PoseObserver {
        virtual void OnPoseAvailable(const Pose& pose) = 0;
    };

   public:
    VIOModule();
    ~VIOModule();

    void OnImageReceived(const ImageData::Ptr imageData) override;

    void SetFrameAdapter(FrameAdapter* adapter);
    void SetPointAdapter(WorldPointAdapter* adapter);

   private:
    bool HaveThingsTodo() override;
    void DoWhatYouNeedToDo() override;
    VIOAlgorithm vio_algorithm_;

    std::vector<PoseObserver*> pose_observers_;
};

}  // namespace DeltaVins
