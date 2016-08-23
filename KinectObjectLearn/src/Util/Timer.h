
#ifndef _Timer_H_
#define _Timer_H_

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

#include <time.h>
#include <string>

namespace Util {

    class Timer {
    public:
        Timer();

        void start(void);
        void stop(void);
        float getNumElapsedSeconds(void);
        float getNumElapsedSecondsSplit(void);

    private:

#ifdef _WIN32
        LARGE_INTEGER start_time, end_time;
        LARGE_INTEGER ticks_per_second;
#else
        struct timeval start_time;
        struct timeval end_time;
#endif

    };

}

#endif


