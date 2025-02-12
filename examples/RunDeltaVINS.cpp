#if USE_ROS2
#include <rclcpp/rclcpp.hpp>
#endif

#include "cmdparser.hpp"
#include "slamAPI.h"
#include "utils/log.h"

void configure_parser(cli::Parser& parser) {
    parser.set_optional<std::string>("c", "", "");
}

int main(int argc, char** argv) {
#ifndef USE_ROS2
    cli::Parser parser(argc, argv);
    configure_parser(parser);
    parser.run_and_exit_if_error();
    auto configFile = parser.get<std::string>("c");
#else
    rclcpp::init(argc, argv);
    std::vector<std::string> non_ros_args =
        rclcpp::remove_ros_arguments(argc, argv);
    if (non_ros_args.size() < 2) {
        LOGE("Didn't get a config file. Exit!");
        exit(1);
    }
    std::string configFile = non_ros_args[1];
#endif

    InitSlamSystem(configFile.c_str());
    StartAndJoin();
    StopSystem();
}