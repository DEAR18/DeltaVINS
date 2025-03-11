#include "precompile.h"
#include "utils/Config.h"
#include "Algorithm/vision/camModel/camModel.h"
#include "Algorithm/vision/camModel/camModel_Equidistant.h"
#include "Algorithm/vision/camModel/camModel_Pinhole.h"
#include "Algorithm/vision/camModel/camModel_RadTan.h"
#include "Algorithm/vision/camModel/camModel_fisheye.h"
#include "utils/log.h"
#include "gtest/gtest.h"
using namespace DeltaVins;

CamModel::Ptr camModel = nullptr;

TEST(CamModel, testReprojectErr) {
    CamModel::Ptr camModel = CamModel::getCamModel();
    camModel->testReprojectErr();
}
