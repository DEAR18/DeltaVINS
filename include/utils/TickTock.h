#pragma once
#include <opencv2/opencv.hpp>
#include <unordered_map>

namespace DeltaVins
{



	class TickTock
	{
	public:
        static cv::TickMeter& get(std::string name);
		static void start(std::string name);
		static void restart(std::string name);
		static void stop(std::string name);
		static void outputResult();
        static void outputResultConsole();
	private:
		static 	std::unordered_map<std::string, cv::TickMeter> m_tickMap;

	};


}