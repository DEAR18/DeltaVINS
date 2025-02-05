#include "framework/VIOModule.h"

#include "IO/dataBuffer/imageBuffer.h"
#include "precompile.h"

namespace DeltaVins {
VIOModule::VIOModule() {}

VIOModule::~VIOModule() {}

void VIOModule::OnImageReceived(const ImageData::Ptr imageData) {
    static int counter = 0;
    counter++;
    if (counter < Config::ImageStartIdx) return;

    static auto& imageBuffer = ImageBuffer::Instance();
    imageBuffer.PushImage(imageData);

    WakeUpMovers();
    if (Config::SerialRun) WaitForThingsToBeDone();
}

void VIOModule::SetFrameAdapter(FrameAdapter* adapter) {
    vio_algorithm_.SetFrameAdapter(adapter);
}

void VIOModule::SetPointAdapter(WorldPointAdapter* adapter) {
    vio_algorithm_.SetWorldPointAdapter(adapter);
}

bool VIOModule::HaveThingsTodo() {
    static auto& imageBuffer = ImageBuffer::Instance();
    return !imageBuffer.empty();
}

void VIOModule::DoWhatYouNeedToDo() {
    TickTock::get("FullFrame").start();
    static auto& imageBuffer = ImageBuffer::Instance();

    auto image = imageBuffer.PopTailImage();

    auto pose = std::make_shared<Pose>();
    vio_algorithm_.AddNewFrame(image, pose);
    if (Config::SerialRun) TellOthersThingsToBeDone();
    TickTock::get("FullFrame").stop();
    if(Config::MaxRunFPS > 0&& Config::SerialRun){
        auto time_cost = TickTock::get("FullFrame").getTimeMilli();
        auto sleep_time = 1000.0 / Config::MaxRunFPS - time_cost;
        if(sleep_time > 0){
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time)));
        }
    }
    TickTock::get("FullFrame").reset();
}
}  // namespace DeltaVins
