#-------------------------------------------------
#
# Project created by QtCreator 2020-12-21T16:46:53
#
#-------------------------------------------------

QT       += core gui
QT += opengl
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets
#QMAKE_CXXFLAGS += -openmp
TARGET = viewer
TEMPLATE = app
OBJECTS_DIR=$${PWD}/build
# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS _USE_MATH_DEFINES
LIBS+=-lopengl32  -lglu32
# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0
CONFIG+=console
CONFIG += c++11
include (../common.pri)
INCLUDEPATH += \
        ..\

HEADERS += \
        interactiveviewerwidget.h \
        mainviewerwidget.h \
        matconvertqimage.h \
        meshparamwidget.h \
        meshviewerwidget.h \
        qglviewerwidget.h \
        surfacemeshprocessing.h \
        thread/modelthread.h \
        thread/threadsafequeue.h \
        thread/threadsynglobalobj.h \
        thread/workingthread.h
SOURCES += \
    interactiveviewerwidget.cpp \
    mainviewerwidget.cpp \
    matconvertqimage.cpp \
    meshparamwidget.cpp \
    meshviewerwidget.cpp \
    qglviewerwidget.cpp \
    surfacemeshprocessing.cpp \
    main.cpp \
    thread/modelthread.cpp \
    thread/threadsynglobalobj.cpp \
    thread/workingthread.cpp

FORMS +=

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target



RESOURCES += \
    viewer.qrc
