
#ifndef _Timer_H_
#define _Timer_H_

#include <sys/time.h>
#include <time.h>

namespace Util {

    class Timer {

      public:

        void start(void);
        void stop(void);
        float getNumElapsedSeconds(void);

      private:

        struct timeval start_time;
        struct timeval end_time;

    };

}

#endif

