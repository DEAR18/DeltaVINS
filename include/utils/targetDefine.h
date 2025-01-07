#pragma once





#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
//define something for Windows (32-bit and 64-bit, this part is common)
#define PLATFORM_WINDOWS
#ifdef _WIN64
   //define something for Windows (64-bit only)
#else
   //define something for Windows (32-bit only)
#endif
#elif __APPLE__
#include <TargetConditionals.h>
#if TARGET_IPHONE_SIMULATOR
// iOS Simulator
#elif TARGET_OS_IPHONE
// iOS device
#define PLATFORM_IOS
#elif TARGET_OS_MAC
#define PLATFORM_MAC
// Other kinds of Mac OS
#else
#   error "Unknown Apple platform"
#endif
#elif __linux__
// linux
#define PLATFORM_LINUX
#elif __unix__ // all unices not caught above
#define PLATFORM_UNIX
// Unix
#elif defined(_POSIX_VERSION)
// POSIX
#else
#   error "Unknown compiler"
#endif



#if defined(__arm__)
#define PLATFORM_ARM
#endif