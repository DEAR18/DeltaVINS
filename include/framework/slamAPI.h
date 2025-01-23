#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void InitSlamSystem(const char* configFile);
void StartAndJoin();
void StopSystem();

#ifdef __cplusplus
}
#endif