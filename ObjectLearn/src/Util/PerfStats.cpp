/*
 * PerfStats.cpp
 *
 *  Created on: 11/03/2009
 *      Author: osushkov
 */

#include "PerfStats.h"
#include <iostream>

using namespace Util;

PerfStats& PerfStats::instance(void){
    static PerfStats perf_stats;
    return perf_stats;
}

PerfStats::PerfStats(){
    total_elapsed_time = 0.0f;
}

void PerfStats::printStats(void){
    return;
    std::map<std::string, Entry>::iterator it = elapsed_time_map.begin();
    while(it != elapsed_time_map.end()){
        std::cout << it->first << "\t\t\t"
                  << it->second.last_time*1000.0f << "\t\t"
                  << it->second.total_time*1000.0f << "\t\t"
                  << 100 * (it->second.total_time/total_elapsed_time)
                  << std::endl;

        ++it;
    }

    std::cout << "Total Elapsed Time: " << total_elapsed_time << std::endl;
}

void PerfStats::addTimeSlice(std::string module_name, float seconds, bool top_level){
    std::map<std::string, Entry>::iterator it = elapsed_time_map.find(module_name);

    if(it == elapsed_time_map.end()){
        elapsed_time_map[module_name].last_time = seconds;
        elapsed_time_map[module_name].total_time = seconds;
    }
    else{
        it->second.last_time = seconds;
        it->second.total_time += seconds;
    }

    if(top_level){
        total_elapsed_time += seconds;
    }
}


