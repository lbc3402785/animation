#include "surfacemeshprocessing.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    //Eigen::initParallel();
    QApplication app(argc, argv);
    QSurfaceFormat format;
    format.setSamples(16);
    QSurfaceFormat::setDefaultFormat(format);
    SurfaceMeshProcessing mainWin;
    mainWin.show();
    return app.exec();
}
