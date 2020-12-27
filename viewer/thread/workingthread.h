#ifndef WORKINGTHREAD_H
#define WORKINGTHREAD_H
#include <QThread>
#include "settings.h"
class WorkingThread : public QThread
{
    Q_OBJECT
public:
    WorkingThread(QObject *parent = nullptr);
    enum WorkingState{CREATED,STARTED,RUNNING,PAUSED,STOPED};
    WorkingState getState() const;
    void setState(const WorkingState &value);

    bool getStopSignal() const;
    void setStopSignal(bool value);

protected:
    bool stopSignal;
    Settings& sets=Settings::instance();
    WorkingState state;
};

#endif // WORKINGTHREAD_H
