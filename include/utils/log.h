#pragma once
#include <cstdio>

#define OUTPUT_INFO 0
#define OUTPUT_WARNING 1
#define OUTPUT_ERROR 1

#define OUTPUT_CONSOLE 1
#define OUTPUT_FILE 0

extern void logInit();
extern void finishLogging();

#if OUTPUT_FILE

extern FILE* infoLog;
extern FILE* warnLog;
extern FILE* errLog;

#endif

#if OUTPUT_INFO
#if OUTPUT_CONSOLE && OUTPUT_FILE
#define LOGI(...)                      \
    {                                  \
        printf("[Info] ");             \
        printf(__VA_ARGS__);           \
        printf("\n");                  \
        fprintf(infoLog, __VA_ARGS__); \
        fprintf(infoLog, "\n");        \
    }
#elif OUTPUT_CONSOLE
#define LOGI(...)            \
    {                        \
        printf("[Info] ");   \
        printf(__VA_ARGS__); \
        printf("\n");        \
    }
#elif OUTPUT_FILE
#define LOGI(...)                      \
    {                                  \
        fprintf(infoLog, __VA_ARGS__); \
        fprintf(infoLog, "\n");        \
    }
#else
#define LOGI(...) void(0)
#endif
#else
#define LOGI(...) void(0)
#endif

#if OUTPUT_WARNING
#if OUTPUT_CONSOLE && OUTPUT_FILE
#define LOGW(...)                      \
    {                                  \
        printf("[Warn] ");             \
        printf(__VA_ARGS__);           \
        ;                              \
        printf("\n");                  \
        fprintf(warnLog, __VA_ARGS__); \
        fprintf(warnLog, "\n");        \
    }
#elif OUTPUT_CONSOLE
#define LOGW(...)            \
    {                        \
        printf("[Warn] ");   \
        printf(__VA_ARGS__); \
        printf("\n");        \
    }
#elif OUTPUT_FILE
#define LOGW(...)                      \
    {                                  \
        fprintf(warnLog, __VA_ARGS__); \
        fprintf(warnLog, "\n");        \
    }
#else
#define LOGW(...) void(0)
#endif
#else
#define LOGW(...) void(0)
#endif

#if OUTPUT_ERROR
#if OUTPUT_CONSOLE && OUTPUT_FILE
#define LOGE(...)                     \
    {                                 \
        printf("[Error] ");           \
        printf(__VA_ARGS__);          \
        printf("\n");                 \
        fprintf(errLog, __VA_ARGS__); \
        fprintf(errLog, "\n");        \
    }
#elif OUTPUT_CONSOLE
#define LOGE(...)            \
    {                        \
        printf("[Error] ");  \
        printf(__VA_ARGS__); \
        printf("\n");        \
    }
#elif OUTPUT_FILE
#define LOGE(...)                 \
    fprintf(errLog, __VA_ARGS__); \
    fprintf(errLog, "\n")
#else
#define LOGE(...) void(0)
#endif
#else
#define LOGE(...) void(0)
#endif