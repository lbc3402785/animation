QT -= gui

CONFIG += c++11 console
CONFIG -= app_bundle
QMAKE_CXXFLAGS += /MP
OBJECTS_DIR=$${PWD}/build
include (../common.pri)
# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS GLM_FORCE_UNRESTRICTED_GENTYPE _USE_MATH_DEFINES
#DEFINES +=BOOST_ALL_DYN_LINK=1




# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


INCLUDEPATH+=..
INCLUDEPATH+=$$TRIMESH_INCLUDE_DIRS
INCLUDEPATH+=d:\soft\glm



#INCLUDEPATH+="C:\Program Files (x86)\tinyobjloader\include" \
#                "C:\Program Files (x86)\xtensor\include" \
#                "C:\Program Files (x86)\xtl\include" \
#                "C:\Program Files (x86)\xsimd\include" \
#                D:\soft\CGAL-4.14.1\auxiliary\gmp\include \
#                D:\soft\CGAL-4.14.1\include \
#                D:\soft\boost_1_69_0

#LIBS += -L$$TRIMESH_LIBRARY_DIRS   -ltrimesh2
#LIBS += -L$$TRIMESH_LIBRARY_DIRS   -lgluit


#LIBS+=-L"C:\Program Files (x86)\tinyobjloader\lib" -ltinyobjloader
#LIBS+=-L"D:\soft\CGAL-4.14.1\auxiliary\gmp\lib" -llibgmp-10
#LIBS+=-L"D:\soft\CGAL-4.14.1\auxiliary\gmp\lib" -llibmpfr-4

SOURCES += \
        test.cpp \
        main.cpp


# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

HEADERS += \
    test.h
