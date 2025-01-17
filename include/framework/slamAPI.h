#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void InitSlamSystem(const char* datasetDir, const char* testName);
void StartAndJoin();
void StopSystem();

#ifdef __cplusplus
}
#endif