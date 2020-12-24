#ifndef INTERACTIVEVIEWERWIDGET_H
#define INTERACTIVEVIEWERWIDGET_H
#include "meshviewerwidget.h"

class InteractiveViewerWidget : public MeshViewerWidget
{
    Q_OBJECT
public:
    InteractiveViewerWidget(QWidget* parent = 0);
    ~InteractiveViewerWidget();
protected:
    void dragEnterEvent(QDragEnterEvent *event);
    void dropEvent(QDropEvent *event);
};

#endif // INTERACTIVEVIEWERWIDGET_H
