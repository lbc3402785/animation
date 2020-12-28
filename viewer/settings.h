#ifndef SETTINGS_H
#define SETTINGS_H
#include <QString>
#include "thread/threadsynglobalobj.h"
const QString SETTINGS_FILE{"AnimationConfig"};
class Settings
{
public:
    static Settings& instance() {static Settings obj; return obj; }
    void saveSettings();
    void reset();


    int getInterval() const;
    void setInterval(int value);
    ThreadSynGlobalObj obj;



private:
    Settings();
    void loadSettings();

    int getDefaultInterval();
    int interval;

};

#endif // SETTINGS_H
