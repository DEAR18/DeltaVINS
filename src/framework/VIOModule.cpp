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
    static auto& imageBuffer = ImageBuffer::Instance();

    auto image = imageBuffer.PopTailImage();

    auto pose = std::make_shared<Pose>();
    vio_algorithm_.AddNewFrame(image, pose);
    if (Config::SerialRun) TellOthersThingsToBeDone();
}
}  // namespace DeltaVins