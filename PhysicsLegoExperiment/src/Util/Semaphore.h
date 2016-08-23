
#ifndef _Semaphore_H_
#define _Semaphore_H_

#include <boost/thread/condition.hpp>
#include <boost/thread/recursive_mutex.hpp>

namespace Util {

    class Semaphore {
      public:

        Semaphore();
        Semaphore(int counter);

        ~Semaphore();

        void wait(void);
        void signal(void);

        int getCount(void) const;

      private:

        int scount;

        boost::recursive_mutex slock;
        boost::condition cvar;

    };

}

#endif

