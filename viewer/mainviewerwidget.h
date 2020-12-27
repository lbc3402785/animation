#ifndef MAINVIEWERWIDGET_H
#define MAINVIEWERWIDGET_H
#include <QWidget>
#include <QDialog>
#include "meshparamwidget.h"
#include "interactiveviewerwidget.h"

class MainViewerWidget : public QDialog
{
    Q_OBJECT

public:
    MainViewerWidget(QWidget *parent = nullptr);
    ~MainViewerWidget();
protected:
    virtual void InitViewerWindow(void);
    virtual void CreateParamWidget(void);
    virtual void CreateViewerDialog(void);
    virtual void OpenMeshGUI(const QString & fname);
    virtual void OpenVideoGUI(const QString & fname);
    virtual void SaveMeshGUI(const QString & fname);
public slots:
    void Open(void);
    void ReadVideo(void);
    void PauseOrResumeVideo();
    void Save(void);
    void ClearMesh(void);
    void Screenshot(void);

    void ShowPoints(void);
    void ShowWireframe(void);
    void ShowHiddenLines(void);
    void ShowFlatLines(void);
    void ShowFlat(void);
    void ShowSmooth(void);
    void Lighting(bool b);
    void DoubleSideLighting(bool b);
    void ShowBoundingBox(bool b);
    void ShowBoundary(bool b);
    void ResetView(void);
    void ViewCenter(void);
    void CopyRotation(void);
    void LoadRotation(void);

signals:
    void haveLoadMesh(QString filePath);
    void havePauseOrResume(bool pause);
protected:
    bool loadmeshsuccess;
private:
    MeshParamWidget* meshparamwidget;
    InteractiveViewerWidget* meshviewerwidget;
};

#endif // MAINVIEWERWIDGET_H
