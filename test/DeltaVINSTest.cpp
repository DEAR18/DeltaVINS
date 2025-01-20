#include <opencv2/opencv.hpp>

#include "cmdparser.hpp"
#include "slamAPI.h"

#if USE_ROS2
#include <rclcpp/rclcpp.hpp>
#endif

void configure_parser(cli::Parser& parser) {
    parser.set_optional<std::string>("o", "", "");
    parser.set_optional<std::string>("d", "", "");
}

int main(int argc, char** argv) {
#ifndef USE_ROS2
    cli::Parser parser(argc, argv);
    configure_parser(parser);
    parser.run_and_exit_if_error();
    auto outputName = parser.get<std::string>("o");
    auto datasetDir = parser.get<std::string>("d");
#else
    rclcpp::init(argc, argv);
    std::string  outputName = "";
    std::string  datasetDir = "";
#endif

    InitSlamSystem(datasetDir.c_str(), outputName.c_str());
    StartAndJoin();
    StopSystem();
}