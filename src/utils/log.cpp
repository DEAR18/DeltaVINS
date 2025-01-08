#include "utils/log.h"

#include "precompile.h"
#include "utils/utils.h"

#if OUTPUT_FILE
FILE *infoLog = nullptr;
FILE *warnLog = nullptr;
FILE *errLog = nullptr;
#endif

void logInit() {
    DeltaVins::existOrMkdir("./Log");
    time_t rawtime;
    struct tm *timeinfo;

    time(&rawtime);
    timeinfo = localtime(&rawtime);
    printf("Current local time and date: %s", asctime(timeinfo));

#if OUTPUT_FILE

    std::string asc = asctime(timeinfo);
    for (auto &c : asc)
        if (c == ':') c = ' ';
    asc.pop_back();
    infoLog = fopen(("./Log/[Info]" + asc + ".txt").c_str(), "w");
    warnLog = fopen(("./Log/[Warn]" + asc + ".txt").c_str(), "w");
    errLog = fopen(("./Log/[Error]" + asc + ".txt").c_str(), "w");
#endif
}

void finishLogging() {
#if OUTPUT_FILE

    fflush(infoLog);
    fflush(warnLog);
    fflush(errLog);
    fclose(infoLog);
    fclose(warnLog);
    fclose(errLog);
    infoLog = nullptr;
    warnLog = nullptr;
    errLog = nullptr;
#endif
}