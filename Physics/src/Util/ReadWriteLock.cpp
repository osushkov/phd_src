
#include "Semaphore.h"
#include "ReadWriteLock.h"

namespace Util {

ReadWriteLock::ReadWriteLock() : r(0), w(0), e(1), nr(0), nw(0), dr(0), dw(0) {
}

ReadWriteLock::~ReadWriteLock(){
}

void ReadWriteLock::getReadLock(void){
    e.wait();
    if(nw > 0 || dw > 0){
        dr++;
        e.signal();
        r.wait();
    }
    
    nr++;
    
    if(dr > 0){
        dr--;
        r.signal();
    }
    else{
        e.signal();
    }
}

void ReadWriteLock::getWriteLock(void){
    e.wait();
    if(nr > 0 || nw > 0){
        dw++;
        e.signal();
        w.wait();
    }

    nw++;
    e.signal();
}

void ReadWriteLock::releaseReadLock(void){
    e.wait();
    nr--;
    if(nr == 0 && dw > 0){
        dw--;
        w.signal();
    }
    else{
        e.signal();
    }
}

void ReadWriteLock::releaseWriteLock(void){
    e.wait();
    nw--;
        
    if(dw > 0){
        dw--;
        w.signal();
    }
    else if(dr > 0){
        dr--;
        r.signal();
    }
    else{
        e.signal();
    }
}

}

