#pragma once


#ifdef __cplusplus
extern "C"{
#endif

	void initSlamSystem(const char* datasetDir, const char* testName);
	void startAndJoin();
	void stopSystem();

#ifdef __cplusplus
}
#endif