#include "Timer.h"

#define MICRO_SECONDS_IN_SECOND 1000000

using namespace Util;

#ifdef _WIN32
Timer::Timer(){
    QueryPerformanceFrequency(&ticks_per_second);
}

void Timer::start(void){
    QueryPerformanceCounter(&start_time);
}

void Timer::stop(void){
    QueryPerformanceCounter(&end_time);
}

float Timer::getNumElapsedSeconds(void){
    return (float)(end_time.QuadPart - start_time.QuadPart)/(float)ticks_per_second.QuadPart;
}

#else

Timer::Timer() {

}

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

#endif

