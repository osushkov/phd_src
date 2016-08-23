
#ifndef _Timer_H_
#define _Timer_H_

#include <sys/time.h>
#include <time.h>
#include <string>

namespace Util {

    class Timer {

      public:
        Timer();
        Timer(std::string module_name);

        void start(void);
        void stop(void);
        float getNumElapsedSeconds(void);

      private:
        const std::string module_name;
        const bool register_perf_stats;


        struct timeval start_time;
        struct timeval end_time;

    };

}

#endif

