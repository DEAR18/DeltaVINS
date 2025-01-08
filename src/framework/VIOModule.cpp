#include "framework/VIOModule.h"

#include "IO/dataBuffer/imageBuffer.h"
#include "precompile.h"

namespace DeltaVins {
VIOModule::VIOModule() {}

VIOModule::~VIOModule() {}

void VIOModule::onImageReceived(const ImageData::Ptr imageData) {
    static int counter = 0;
    counter++;
    if (counter < Config::ImageStartIdx) return;

    static auto& imageBuffer = ImageBuffer::getInstance();
    imageBuffer.pushImage(imageData);

    wakeUpMovers();
    if (Config::SerialRun) waitForThingsToBeDone();
}

void VIOModule::setFrameAdapter(FrameAdapter* adapter) {
    m_vioAlgorithm.setFrameAdapter(adapter);
}

void VIOModule::setPointAdapter(WorldPointAdapter* adapter) {
    m_vioAlgorithm.setWorldPointAdapter(adapter);
}

bool VIOModule::haveThingsTodo() {
    static auto& imageBuffer = ImageBuffer::getInstance();
    return !imageBuffer.empty();
}

void VIOModule::doWhatYouNeedToDo() {
    static auto& imageBuffer = ImageBuffer::getInstance();

    auto image = imageBuffer.popTailImage();

    auto pose = std::make_shared<Pose>();
    m_vioAlgorithm.addNewFrame(image, pose);
    if (Config::SerialRun) tellOthersThingsToBeDone();
}
}  // namespace DeltaVins