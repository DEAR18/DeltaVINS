#pragma once

#ifdef __cplusplus
extern "C" {
#endif

bool InitSlamSystem(const char* configFile);
void StartAndJoin();
void StopSystem();

#ifdef __cplusplus
}
#endif