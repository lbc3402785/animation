OPENCV_INCLUDE_DIRS=D:\soft\opencv3\build\include
OPENCV_LIBRARY_DIRS=D:\soft\opencv3\build\x64\vc15\lib
OPENMESH_INCLUDE_DIRS="D:\soft\OpenMesh8.0\include"
OPENMESH_LIBRARY_DIRS="D:\soft\OpenMesh8.0\lib"

ZLIB_INCLUDE_DIRS="D:\soft\opencv3\sources\3rdparty\zlib"
INCLUDEPATH+=$$OPENCV_INCLUDE_DIRS
INCLUDEPATH+=$$OPENMESH_INCLUDE_DIRS
INCLUDEPATH+=$$ZLIB_INCLUDE_DIRS
INCLUDEPATH+=D:\soft\eigen-eigen-323c052e1731
INCLUDEPATH+=$$(DLIB_ROOT)\include

INCLUDEPATH+="C:\Program Files (x86)\Ceres\include" \
                 "C:\Program Files (x86)\glog\include" \
                 "C:\Program Files (x86)\gflags\include"

LIBS+=-L$$OPENCV_LIBRARY_DIRS -lopencv_world344
LIBS +=-L$$OPENMESH_LIBRARY_DIRS -lOpenMeshCore
LIBS+=-L$$(DLIB_ROOT)\lib -ldlib19.16.99_release_64bit_msvc1900

LIBS+=-L"C:\Program Files (x86)\Ceres\lib" -lceres
LIBS+=-L"C:\Program Files (x86)\glog\lib" -lglog
LIBS+=-L"C:\Program Files (x86)\gflags\lib" -lgflags
HEADERS += \
    ../modelsequence.h \
    ../MeshDefinition.h \
    ../common/imageprocess.h \
    ../denselaplace.h \
    ../meshtools.h \
    ../process/pointcloudregister.h \
    ../setoperator.h \
    ../sparselaplace.h \
    ../src/Dlib.h \
    ../src/MMSolver.h \
    ../src/Poisson.h \
    ../src/ceresnonlinear.hpp \
    ../src/cnpy.h \
    ../src/facemodel.h \
    ../src/preprocess.h \
    ../util/eigenfunctions.h \
    ../uv_mapper/half_edge_mesh.hpp \
    ../uv_mapper/vec.hpp

SOURCES +=\
        ../modelsequence.cpp \
        ../common/imageprocess.cpp \
        ../meshtools.cpp \
        ../src/Dlib.cpp \
        ../src/MMSolver.cpp \
        ../src/cnpy.cpp \
        ../src/facemodel.cpp \
        ../src/preprocess.cpp \
        ../uv_mapper/half_edge_mesh.cpp
