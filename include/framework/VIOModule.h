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
        virtual void onPoseAvailable(const Pose& pose) = 0;
    };

   public:
    VIOModule();
    ~VIOModule();

    void onImageReceived(const ImageData::Ptr imageData) override;

    void setFrameAdapter(FrameAdapter* adapter);
    void setPointAdapter(WorldPointAdapter* adapter);

   private:
    bool haveThingsTodo() override;
    void doWhatYouNeedToDo() override;
    VIOAlgorithm m_vioAlgorithm;

    std::vector<PoseObserver*> m_v_PoseObservers;
};

}  // namespace DeltaVins
