
#include "Timer.h"

#define MICRO_SECONDS_IN_SECOND 1000000

namespace Util {

void Timer::start(void){
    gettimeofday(&start_time, NULL);
}

void Timer::stop(void){
    gettimeofday(&end_time, NULL);
}

float Timer::getNumElapsedSeconds(void){
    unsigned int seconds = end_time.tv_sec - start_time.tv_sec;
    unsigned int mseconds = seconds*MICRO_SECONDS_IN_SECOND - start_time.tv_usec + end_time.tv_usec;

    return (float)mseconds/(float)MICRO_SECONDS_IN_SECOND;
}

}
