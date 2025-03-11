#include <gtest/gtest.h>

#include "Algorithm/vision/camModel/camModel_Equidistant.h"
#include "utils/Config.h"

TEST(EquidistantModel, TestMonocularModelPrecision) {
    int width = 1440;
    int height = 1080;
    float fx = 698.169f;
    float fy = 698.515f;
    float cx = 712.765f;
    float cy = 560.575f;
    float k1 = -0.0411423f;
    float k2 = -0.00242177f;
    float k3 = 0.00340036f;
    float k4 = -0.00185288f;
    DeltaVins::EquiDistantModel equidistantModel(width, height, fx, fy, cx, cy,
                                                 k1, k2, k3, k4);

    DeltaVins::Config::ResultOutputPath = "./";
    float model_err = equidistantModel.testModelPrecision(false);

    EXPECT_NEAR(model_err, 0.0f, 1e-3f);
}

TEST(EquidistantModel, TestStereoModelPrecision) {
    int width = 1440;
    int height = 1080;
    float fx = 698.169f;
    float fy = 698.515f;
    float cx = 712.765f;
    float cy = 560.575f;
    float k1 = -0.0411423f;
    float k2 = -0.00242177f;
    float k3 = 0.00340036f;
    float k4 = -0.00185288f;
    float fx_right = 697.491f;
    float fy_right = 696.917f;
    float cx_right = 719.172f;
    float cy_right = 543.740f;
    float k1_right = -0.0441783f;
    float k2_right = 0.00623428f;
    float k3_right = -0.00281530f;
    float k4_right = -0.000565212f;
    DeltaVins::EquiDistantModel equidistantModel(
        width, height, fx, fy, cx, cy, k1, k2, k3, k4, fx_right, fy_right,
        cx_right, cy_right, k1_right, k2_right, k3_right, k4_right);

    DeltaVins::Config::ResultOutputPath = "./";
    float left_model_err = equidistantModel.testModelPrecision(false);
    float right_model_err = equidistantModel.testModelPrecision(true);

    EXPECT_NEAR(left_model_err, 0.0f, 1e-3f);
    EXPECT_NEAR(right_model_err, 0.0f, 1e-3f);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
