#include "precompile.h"
#include <utils/TickTock.h>



namespace DeltaVins
{

	std::unordered_map<std::string, cv::TickMeter> TickTock::m_tickMap;

	void TickTock::start(std::string name)
	{
		m_tickMap[name].start();
	}

	void TickTock::stop(std::string name)
	{
		m_tickMap[name].stop();
	}

	void TickTock::outputResult()
	{
		FILE* fout = fopen("Time.txt", "w");
		for (auto&k:m_tickMap)
		{
			fprintf(fout, "%s:", k.first.c_str());
			fprintf(fout, "Mean time:%f ms\n", k.second.getTimeMilli() / k.second.getCounter());
		}
		fclose(fout);
	}

    void TickTock::outputResultConsole() {
	    if(!Config::NoDebugOutput) {
            for (auto &k:m_tickMap) {
                printf("%s:", k.first.c_str());
                printf("Mean time:%f ms\n", k.second.getTimeMilli() / k.second.getCounter());
            }
        }
    }

    void TickTock::restart(std::string name) {
        m_tickMap[name].reset();

    }

    cv::TickMeter& TickTock::get(std::string name) {
        return m_tickMap[name];
    }
}