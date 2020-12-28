#include "settings.h"
#include <QSettings>
#include <QDir>
#include <QStandardPaths>
Settings::Settings()
{
    loadSettings();
}

void Settings::loadSettings()
{
    QSettings settings(SETTINGS_FILE);
    interval=settings.value("interval", getDefaultInterval()).toInt();
}







int Settings::getDefaultInterval()
{
    return 5;
}

int Settings::getInterval() const
{
    return interval;
}

void Settings::setInterval(int value)
{
    interval = value;
}


void Settings::saveSettings()
{
    QSettings settings(SETTINGS_FILE);
    settings.setValue("interval", interval);
    settings.sync();
}

void Settings::reset()
{
    interval=getDefaultInterval();
}


