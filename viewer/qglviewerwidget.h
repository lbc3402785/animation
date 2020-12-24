#ifndef QGLVIEWERWIDGET_H
#define QGLVIEWERWIDGET_H
#include <OpenMesh/Core/Geometry/VectorT.hh>
#include <QGLWidget>
namespace Ui {
class QGLViewerWidget;
}

class QGLViewerWidget :public QGLWidget
{
    Q_OBJECT

public:
    explicit QGLViewerWidget(QWidget *parent = nullptr);
    ~QGLViewerWidget();

private:
    void Init(void);
public:
    QSize minimumSizeHint(void) const override;
    QSize sizeHint(void) const override;
    const double & Radius(void) const;
    const OpenMesh::Vec3f & Center(void) const;
    const double* GetModelviewMatrix(void) const;
    void ResetModelviewMatrix(void);
    void CopyModelViewMatrix(void);
    void LoadCopyModelViewMatrix(void);
    const double* GetProjectionMatrix(void) const;

    enum ProjectionMode{ PERSPECTIVE, ORTHOGRAPHIC };
    void SetProjectionMode(const ProjectionMode &pm);
    const ProjectionMode & GetProjectionMode(void) const;

    enum DrawMode{ POINTS, WIREFRAME, HIDDENLINES, FLATLINES, FLAT, SMOOTH };
    void SetDrawMode(const DrawMode &dm);
    const DrawMode& GetDrawMode(void) const;

protected:
    enum MaterialType { MaterialDefault, MaterialGold, MaterialSilver, MaterialEmerald, MaterialTin };
    void SetMaterial(const MaterialType & mattype = MaterialDefault) const;
    void SetDefaultLight(void) const;
    void initializeGL(void) override;
    void resizeGL(int w, int h) override;
    void paintGL(void) override;
    virtual void DrawScene(void);
    virtual void mousePressEvent(QMouseEvent*) override;
    virtual void mouseMoveEvent(QMouseEvent*) override;
    virtual void mouseReleaseEvent(QMouseEvent*) override;
    virtual void wheelEvent(QWheelEvent*) override;
    virtual void keyPressEvent(QKeyEvent*) override;
    virtual void keyReleaseEvent(QKeyEvent*) override;
private:
    void Translation(const QPoint & p);
    void Translate(const OpenMesh::Vec3f & trans);
    void Rotation(const QPoint & p);
    void Rotate(const OpenMesh::Vec3f & axis, const double & angle);
    bool MapToSphere(const QPoint & point, OpenMesh::Vec3f & result);
    void UpdateProjectionMatrix(void);
public:
    void SetScenePosition(const OpenMesh::Vec3f & c, const double & r);
    void ViewAll(void);
protected:
    DrawMode drawmode;
    ProjectionMode projectionmode;
    double windowleft;
    double windowright;
    double windowtop;
    double windowbottom;
    Qt::MouseButton mousemode;
    OpenMesh::Vec3f center;
    double radius;
    std::vector<double> projectionmatrix;
    std::vector<double> modelviewmatrix;
    std::vector<double> copymodelviewmatrix;
    QPoint lastpoint2;
    OpenMesh::Vec3f lastpoint3;
    bool lastpointok;
private:
    static const double trackballradius;

};

#endif // QGLVIEWERWIDGET_H
