#ifndef MODELTHREAD_H
#define MODELTHREAD_H
#include <mutex>
#include "workingthread.h"
#include "modelsequence.h"
#include "threadsafequeue.h"
class ModelThread:public WorkingThread
{
    Q_OBJECT
public:
    ModelThread(std::shared_ptr<ModelSequence>& solverPtr,std::shared_ptr<ThreadSafeQueue<cv::Mat>> &imageQueue,
                std::shared_ptr<ThreadSafeQueue< std::tuple<MatF,MatF,cv::Mat> > >&resultQueue,QObject *parent = nullptr);
    bool getFirst() const;
    void setFirst(bool value);

    bool getPause() const;
    void setPause(bool value);

    bool getStopSignal() const;
    void setStopSignal(bool value);

private:
    std::shared_ptr<ModelSequence> solverPtr;
    std::shared_ptr<ThreadSafeQueue<cv::Mat>> imageQueue;
    std::shared_ptr<ThreadSafeQueue< std::tuple<MatF,MatF,cv::Mat> > > resultQueue;

    // QThread interface
protected:
    void run();
    bool stopSignal;
    bool pause;
    bool first;
    int interval;
    WorkingState state;
    cv::Mat image;
};

#endif // MODELTHREAD_H
