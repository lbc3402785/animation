#ifndef THREADSYNGLOBALOBJ_H
#define THREADSYNGLOBALOBJ_H

#include <mutex>
class ThreadSynGlobalObj
{
public:
    ThreadSynGlobalObj();
    mutable  std::mutex cameraTheadMutex;
    mutable  std::mutex udpTheadMutex;
};

#endif // THREADSYNGLOBALOBJ_H
