#include "mainviewerwidget.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
MainViewerWidget::MainViewerWidget(QWidget *parent)
{
    InitViewerWindow();
}

MainViewerWidget::~MainViewerWidget()
{

}

void MainViewerWidget::InitViewerWindow()
{
    CreateViewerDialog();
    CreateParamWidget();
    QHBoxLayout* main_layout = new QHBoxLayout();
    main_layout->addWidget(meshparamwidget);
    main_layout->addWidget(meshviewerwidget, 1);
    this->setLayout(main_layout);
}

void MainViewerWidget::CreateParamWidget()
{
    meshparamwidget = new MeshParamWidget();
    //connect(meshparamwidget, SIGNAL(PrintInfoSignal()), meshviewerwidget, SLOT(PrintMeshInfo()));
}

void MainViewerWidget::CreateViewerDialog()
{
    meshviewerwidget = new InteractiveViewerWidget(NULL);
    meshviewerwidget->setAcceptDrops(true);
    //connect(meshviewerwidget, SIGNAL(LoadMeshOKSignal(bool, QString)), SLOT(LoadMeshFromInner(bool, QString)));
}
void MainViewerWidget::OpenMeshGUI(const QString & fname)
{
    if (fname.isEmpty() || !meshviewerwidget->LoadMesh(fname.toStdString()))
    {
        QString msg = "Cannot read mesh from file:\n '" + fname + "'";
        QMessageBox::critical(NULL, windowTitle(), msg);
    }
    else
    {
        loadmeshsuccess = true;
        emit(haveLoadMesh(fname));
    }
}

void MainViewerWidget::OpenVideoGUI(const QString &fname)
{
    if (fname.isEmpty())
    {
        QString msg = "Cannot read mesh from file:\n '" + fname + "'";
        QMessageBox::critical(NULL, windowTitle(), msg);
    }else{
        meshviewerwidget->ReadVideo(fname.toStdString());
    }
}

void MainViewerWidget::SaveMeshGUI(const QString & fname)
{
    if (fname.isEmpty() || !meshviewerwidget->SaveMesh(fname.toStdString()))
    {
        QString msg = "Cannot read mesh from file:\n '" + fname + "'";
        QMessageBox::critical(NULL, windowTitle(), msg);
    }
}
void MainViewerWidget::Open(void)
{
    QString fileName = QFileDialog::getOpenFileName(this,
        tr("Open mesh file"),
        tr(""),
        tr("Mesh Files (*.obj *.off *.ply *.stl);;"
        "OFF Files (*.off);;"
        "OBJ Files (*.obj);;"
        "PLY Files (*.ply);;"
        "STL Files (*.stl);;"
        "All Files (*)"));
    if (!fileName.isEmpty())
    {
        OpenMeshGUI(fileName);
    }
}
void MainViewerWidget::ReadVideo(void)
{
    QString fileName = QFileDialog::getOpenFileName(this,
        tr("Open video file"),
        tr(""),
        tr("Mesh Files (*.mp4 *.avi *.wmv *.mpg);;"
        "mp4 Files (*.mp4);;"
        "avi Files (*.avi);;"
        "wmv Files (*.wmv);;"
        "mpg Files (*.mpg)"));
    if (!fileName.isEmpty())
    {
        OpenVideoGUI(fileName);
    }
}

void MainViewerWidget::Save(void)
{
    QString fileName = QFileDialog::getSaveFileName(this,
        tr("Save mesh file"),
        tr("untitled.obj"),
        tr("OBJ Files (*.obj);;"
        "OFF Files (*.off);;"
        "PLY Files (*.ply);;"
        "STL Files (*.stl);;"
        "All Files (*)"));
    if (!fileName.isEmpty())
    {
        SaveMeshGUI(fileName);
    }
}

void MainViewerWidget::ClearMesh(void)
{
    if (loadmeshsuccess)
    {
        loadmeshsuccess = false;
        meshviewerwidget->Clear();
    }
}

void MainViewerWidget::Screenshot(void)
{
    meshviewerwidget->ScreenShot();
}
void MainViewerWidget::ShowPoints(void)
{
    meshviewerwidget->SetDrawMode(InteractiveViewerWidget::POINTS);
}

void MainViewerWidget::ShowWireframe(void)
{
    meshviewerwidget->SetDrawMode(InteractiveViewerWidget::WIREFRAME);
}

void MainViewerWidget::ShowHiddenLines(void)
{
    meshviewerwidget->SetDrawMode(InteractiveViewerWidget::HIDDENLINES);
}

void MainViewerWidget::ShowFlatLines(void)
{
    meshviewerwidget->SetDrawMode(InteractiveViewerWidget::FLATLINES);
}

void MainViewerWidget::ShowFlat(void)
{
    meshviewerwidget->SetDrawMode(InteractiveViewerWidget::FLAT);
}

void MainViewerWidget::ShowSmooth(void)
{
    meshviewerwidget->SetDrawMode(InteractiveViewerWidget::SMOOTH);
}

void MainViewerWidget::Lighting(bool b)
{
    meshviewerwidget->EnableLighting(b);
}

void MainViewerWidget::DoubleSideLighting(bool b)
{
    meshviewerwidget->EnableDoubleSide(b);
}

void MainViewerWidget::ShowBoundingBox(bool b)
{
    meshviewerwidget->SetDrawBoundingBox(b);
}

void MainViewerWidget::ShowBoundary(bool b)
{
    meshviewerwidget->SetDrawBoundary(b);
}

void MainViewerWidget::ResetView(void)
{
    meshviewerwidget->ResetView();
}

void MainViewerWidget::ViewCenter(void)
{
    meshviewerwidget->ViewCenter();
}

void MainViewerWidget::CopyRotation(void)
{
    meshviewerwidget->CopyRotation();
}

void MainViewerWidget::LoadRotation(void)
{
    meshviewerwidget->LoadRotation();
}
