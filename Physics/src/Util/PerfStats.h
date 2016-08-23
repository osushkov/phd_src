/*
 * PerfStats.h
 *
 *  Created on: 11/03/2009
 *      Author: osushkov
 */

#ifndef PERFSTATS_H_
#define PERFSTATS_H_

#include <string>
#include <map>

namespace Util {

    class PerfStats {
      public:
        static PerfStats& instance(void);

        void printStats(void);
        void addTimeSlice(std::string module_name, float seconds, bool top_level=false);

      private:
        struct Entry {
            float total_time;
            float last_time;
        };

        std::map<std::string, Entry> elapsed_time_map;
        float total_elapsed_time; // the total recorded time, used for percentage calc.

        PerfStats();
    };

}

#endif /* PERFSTATS_H_ */
