#ifndef MODELTHREAD_H
#define MODELTHREAD_H
#include "workingthread.h"
#include "modelsequence.h"
#include "threadsafequeue.h"
class ModelThread:public WorkingThread
{
    Q_OBJECT
public:
    ModelThread(std::shared_ptr<ModelSequence>& solverPtr,std::shared_ptr<ThreadSafeQueue<cv::Mat>> &imageQueue,QObject *parent = nullptr);
private:
    std::shared_ptr<ModelSequence> solverPtr;
    std::shared_ptr<ThreadSafeQueue<cv::Mat>> imageQueue;
};

#endif // MODELTHREAD_H
