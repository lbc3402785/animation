#ifndef MESHVIEWERWIDGET_H
#define MESHVIEWERWIDGET_H

#include <QString>
#include "common/imageprocess.h"
#include "MeshDefinition.h"
#include "meshtools.h"
#include "QGLViewerWidget.h"
#include <deque>
#include <QOpenGLTexture>
#include "modelsequence.h"
class MeshViewerWidget : public QGLViewerWidget
{
    Q_OBJECT
public:
    std::shared_ptr<ModelSequence> modelPtr;
    bool isUpdate = false;
    int timerId;
    int headOffset = 46398;
    int headTriOffset = 92356;
    std::deque<Mesh> meshQueue;
    std::deque<cv::Mat> imageQueue;
    std::deque<cv::Mat> faceTextureQueue;
    std::deque<cv::Mat> headTextureQueue;
    QOpenGLTexture *textures[2] = { nullptr, nullptr };
    //GLuint textures[2] = { 0, 0 };
    cv::Mat pic;
    VideoCapture sequence;
    int count;
    int cur;
//    cv::Mat face;
//    cv::Mat head;
    MeshViewerWidget(QWidget* parent = 0);
    virtual ~MeshViewerWidget(void);
    bool LoadMesh(const std::string & filename);
    void ReadVideo(const std::string & dir);
    void Clear(void);
    void UpdateMesh(void);
    bool SaveMesh(const std::string & filename);
    bool ScreenShot(void);
    void SetDrawBoundingBox(bool b);
    void SetDrawBoundary(bool b);
    void EnableLighting(bool b);
    void EnableDoubleSide(bool b);
    void ResetView(void);
    void ViewCenter(void);
    void CopyRotation(void);
    void LoadRotation(void);

signals:
    void LoadMeshOKSignal(bool, QString);
public slots:
    void PrintMeshInfo(void);
    void play();
protected:
    virtual void DrawScene(void) override;
    void DrawSceneMesh(void);
    void paintGL(void);
    void timerEvent(QTimerEvent *);
private:
    QTimer* timer;
    bool first;
    cv::Mat image;
    std::deque<MatF> models;
    std::deque<MatF> uvs;
    std::deque<Mat> faceTextures;
    float* pointData;
    float* uvData;
    void makeMesh(MatF& Face,MatI& TRI);
    void DrawPoints(void) const;
    void DrawWireframe(void) const;
    void DrawHiddenLines(void) const;
    void DrawFlatLines(void) const;
    void DrawFlat(void) const;
    void DrawSmooth(void) const;
    void DrawBoundingBox(void) const;
    void DrawBoundary(void) const;
protected:
    bool isReaded;
    Mesh mesh;
    QString strMeshFileName;
    QString strMeshBaseName;
    QString strMeshPath;
    Mesh::Point ptMin;
    Mesh::Point ptMax;
    bool isEnableLighting;
    bool isTwoSideLighting;
    bool isDrawBoundingBox;
    bool isDrawBoundary;
};

#endif // MESHVIEWERWIDGET_H
