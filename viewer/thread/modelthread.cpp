#include "modelthread.h"
#include "threadsynglobalobj.h"
ModelThread::ModelThread(std::shared_ptr<ModelSequence> &solverPtr, std::shared_ptr<ThreadSafeQueue<Mat> > &imageQueue,
                         std::shared_ptr<ThreadSafeQueue< std::tuple<MatF,MatF,cv::Mat> > >&resultQueue,
                         QObject *parent)
    :solverPtr(solverPtr),imageQueue(imageQueue),resultQueue(resultQueue), WorkingThread(parent)
{
    first=true;
    stopSignal=false;
    pause=false;
    interval=5;
}

void ModelThread::run()
{
    state=RUNNING;
    while (!stopSignal) {
        if (isInterruptionRequested())
            return;
        if(pause){
            msleep(interval*2);
        }else{
            imageQueue->WaitAndPop(image);
            std::deque<std::tuple<MatF,MatF,cv::Mat>>& results=solverPtr->readOnePic(image,first,false);
            while(!results.empty()){
                resultQueue->TryPush(results.front());
                results.pop_front();
            }
            msleep(interval);
        }
    }
}

bool ModelThread::getStopSignal() const
{
    return stopSignal;
}

void ModelThread::setStopSignal(bool value)
{
    stopSignal = value;
}

bool ModelThread::getPause() const
{
    return pause;
}

void ModelThread::setPause(bool value)
{
    pause = value;
}

bool ModelThread::getFirst() const
{
    return first;
}

void ModelThread::setFirst(bool value)
{
    first = value;
}
