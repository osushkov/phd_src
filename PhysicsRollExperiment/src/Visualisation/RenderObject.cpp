
#include "RenderObject.h"
#include "../Util/Semaphore.h"

static unsigned cur_id;
static Util::Semaphore lock(1);

unsigned RenderObject::getNewId(void){
    unsigned result;
    lock.wait();
    result = cur_id++;
    lock.signal();
    return result;
}