#pragma once
#include <opencv2/opencv.hpp>
#include <unordered_map>

namespace DeltaVins {

class TickTock {
   public:
    static cv::TickMeter& get(std::string name);
    static void Start(std::string name);
    static void restart(std::string name);
    static void Stop(std::string name);
    static void outputResult(const std::string& output_file);
    static void outputResultConsole();

   private:
    static std::unordered_map<std::string, cv::TickMeter> m_tickMap;
};

}  // namespace DeltaVins