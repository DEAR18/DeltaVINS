#include "slamAPI.h"
#include "cmdparser.hpp"
#include <opencv2/opencv.hpp>


void configure_parser(cli::Parser& parser) {
	parser.set_optional<std::string>("o", "", "");
	parser.set_optional<std::string>("d", "", "");
}

int main(int argc,char**argv)
{
	cli::Parser parser(argc, argv);
	configure_parser(parser);
	parser.run_and_exit_if_error(); 
	auto outputName = parser.get<std::string>("o");
	auto datasetDir = parser.get<std::string>("d");
	InitSlamSystem(datasetDir.c_str(),outputName.c_str());
	StartAndJoin();
	StopSystem();
}