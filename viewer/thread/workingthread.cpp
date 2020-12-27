#include "workingthread.h"

WorkingThread::WorkingThread(QObject *parent):QThread (parent)
{
    state=CREATED;
}

WorkingThread::WorkingState WorkingThread::getState() const
{
    return state;
}

void WorkingThread::setState(const WorkingState &value)
{
    state = value;
}

bool WorkingThread::getStopSignal() const
{
    return stopSignal;
}

void WorkingThread::setStopSignal(bool value)
{
    stopSignal = value;
}
