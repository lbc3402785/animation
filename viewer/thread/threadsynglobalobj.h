#ifndef THREADSYNGLOBALOBJ_H
#define THREADSYNGLOBALOBJ_H

#include <mutex>
class ThreadSynGlobalObj
{
public:
    ThreadSynGlobalObj();
    mutable  std::mutex modelTheadMutex;
};

#endif // THREADSYNGLOBALOBJ_H
