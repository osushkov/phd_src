
#ifndef _ReadWriteLock_H_
#define _ReadWriteLock_H_

#include "Semaphore.h"

namespace Util {

    class ReadWriteLock {
      public:
    
        ReadWriteLock();
        ~ReadWriteLock();

        void getReadLock(void);
        void getWriteLock(void);

        void releaseReadLock(void);
        void releaseWriteLock(void);

      private:

        Semaphore r, w, e;
        int nr, nw;
        int dr, dw;
    
    };
}

#endif

