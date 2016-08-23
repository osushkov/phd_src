
#include "Timer.h"
#include "PerfStats.h"

#define MICRO_SECONDS_IN_SECOND 1000000

using namespace Util;

Timer::Timer() :
    register_perf_stats(false) {

}

Timer::Timer(std::string module_name) :
    module_name(module_name),
    register_perf_stats(true) {

}

void Timer::start(void){
    gettimeofday(&start_time, NULL);
}

void Timer::stop(void){
    gettimeofday(&end_time, NULL);

    if(register_perf_stats){
        PerfStats::instance().addTimeSlice(module_name, getNumElapsedSeconds());
    }
}

float Timer::getNumElapsedSeconds(void){
    unsigned int seconds = end_time.tv_sec - start_time.tv_sec;
    unsigned int mseconds = seconds*MICRO_SECONDS_IN_SECOND - start_time.tv_usec + end_time.tv_usec;

    return (float)mseconds/(float)MICRO_SECONDS_IN_SECOND;
}

