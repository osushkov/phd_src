
#include <boost/thread/condition.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include "Semaphore.h"
#include <cassert>

namespace Util {

Semaphore::Semaphore(){
    scount = 0;
}

Semaphore::Semaphore(int counter){
    scount = counter;
}

Semaphore::~Semaphore(){
}

void Semaphore::wait(void){
    assert(scount >= 0);

    {
        boost::recursive_mutex::scoped_lock lock(slock);
        while (scount == 0){
            cvar.wait(lock);
        }
        assert(scount > 0);
        scount--;
    }
    
}

void Semaphore::signal(void){
    assert(scount >= 0);

    {
        boost::recursive_mutex::scoped_lock lock(slock);
        scount++;
        assert(scount >= 0);
        cvar.notify_one();
    }
}

} // namespace Util

