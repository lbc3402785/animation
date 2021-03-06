#include "meshviewerwidget.h"
#include <QtCore>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include "matconvertqimage.h"
MeshViewerWidget::MeshViewerWidget(QWidget* parent)
    : QGLViewerWidget(parent),
      ptMin(0.0),
      ptMax(0.0),
      isEnableLighting(true),
      isTwoSideLighting(false),
      isDrawBoundingBox(false),
      isDrawBoundary(false)
{
    //timerId = startTimer(50);
    ModelSequence model;
    model.W=2048;
    model.H=2048;
    model.Init("..\\modelsequence\\data\\WomenFullHead1216.npz","..\\modelsequence\\data\\shape_predictor_68_face_landmarks.dat",
               "..\\modelsequence\\data\\WomanFace.png","..\\modelsequence\\data\\WomanHead.jpg","",
               "..\\modelsequence\\data\\WomanMask1.png","..\\modelsequence\\data\\women.ini");
    modelPtr=std::make_shared<ModelSequence>(model);
    headOffset=modelPtr->solver.FMFull.HEAD_OFFSET(0,0);
    headTriOffset=modelPtr->solver.FMFull.TRIUV_FACE.rows();
    Clear();
    isReaded=false;
}

MeshViewerWidget::~MeshViewerWidget(void)
{
}

void MeshViewerWidget::makeMesh(MatF& Face,MatI& TRI)
{
    int nvertices=Face.rows();

    mesh.resize(nvertices,0,0);

    float* data=const_cast<float*>(mesh.point(mesh.vertices_begin()).data());
    memcpy(data,Face.data(),sizeof(float)*3*mesh.n_vertices());
    int nfaces=TRI.rows();
    std::vector<Mesh::VertexHandle> face_vhandles;
    for(int k=0;k<nfaces;k++){
        face_vhandles.clear();
        face_vhandles.emplace_back(modelPtr->solver.FMFull.TRI(k,0));
        face_vhandles.emplace_back(modelPtr->solver.FMFull.TRI(k,1));
        face_vhandles.emplace_back(modelPtr->solver.FMFull.TRI(k,2));
        mesh.add_face(face_vhandles);
    }
}

bool MeshViewerWidget::LoadMesh(const std::string & filename)
{
    Clear();
    bool read_OK = MeshTools::ReadMesh(mesh, filename);
    int index=filename.find_last_of(".");
    std::string baseFileName=filename.substr(0,index);
    std::string facePicPath = baseFileName  + "Face.png";
    //std::cout<<facePicPath<<std::endl;
    std::string headPicPath = baseFileName  + "Head.png";
    if(textures[0]){
        delete textures[0];
    }
    QImage face;
    if(face.load((QString::fromStdString(facePicPath)))){
        textures[0] = new QOpenGLTexture(face);
        textures[0]->setMinificationFilter(QOpenGLTexture::Nearest);
        textures[0]->setMagnificationFilter(QOpenGLTexture::Linear);
        textures[0]->setWrapMode(QOpenGLTexture::Repeat);
    }
    if(textures[1]){
        delete textures[1];
    }
    QImage head;
    if(head.load((QString::fromStdString(headPicPath)))){
        textures[1] = new QOpenGLTexture(head);
        textures[1]->setMinificationFilter(QOpenGLTexture::Nearest);
        textures[1]->setMagnificationFilter(QOpenGLTexture::Linear);
        textures[1]->setWrapMode(QOpenGLTexture::Repeat);
    }
    glEnable( GL_TEXTURE_2D );
    glShadeModel( GL_SMOOTH );
    //        glClearColor( 0.0, 0.0, 0.0, 0.5 );
    //        glClearDepth( 1.0 );
    glEnable( GL_DEPTH_TEST );
    glDepthFunc( GL_LEQUAL );
    glHint( GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST );
    glPolygonMode( GL_BACK, GL_FILL );
    glPolygonMode( GL_FRONT, GL_FILL );
    //std::cout<<headPicPath<<std::endl;
    std::cout << "Load mesh from file " << filename << std::endl;
    if (read_OK)
    {

        /*Mesh::VertexIter v_it;
        for (v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
        {

            if (mesh.has_vertex_texcoords2D())
            {
                auto &tex = mesh.texcoord2D(*v_it);
                std::cout<< v_it ->idx()<<":"<<tex[0]<<","<<tex[1]<<std::endl;
            }
        }
        std::cout << "000" << std::endl;*/
        strMeshFileName = QString::fromStdString(filename);
        QFileInfo fi(strMeshFileName);
        strMeshPath = fi.path();
        strMeshBaseName = fi.baseName();
        UpdateMesh();
        update();
        return true;
    }
    return false;
}

void MeshViewerWidget::ReadVideo(const std::string &videoPath)
{
    if(!isReaded){
        mesh.clear();
        makeMesh(modelPtr->solver.FMFull.Face,modelPtr->solver.FMFull.TRI);
        UpdateMesh();
        update();
        using namespace std;
        using namespace cv;
        pointData=const_cast<float*>(mesh.point(mesh.vertices_begin()).data());
        uvData=const_cast<float*>(mesh.texcoord2D(mesh.vertices_begin()).data());
        if(textures[1]){
            delete textures[1];
        }
        QImage head=MatConvertQImage::Mat2QImage(modelPtr->HeadTexture);
        textures[1] = new QOpenGLTexture(head);
        textures[1]->setMinificationFilter(QOpenGLTexture::Nearest);
        textures[1]->setMagnificationFilter(QOpenGLTexture::Linear);
        textures[1]->setWrapMode(QOpenGLTexture::Repeat);
        glEnable( GL_TEXTURE_2D );
        glShadeModel( GL_SMOOTH );
        //        glClearColor( 0.0, 0.0, 0.0, 0.5 );
        //        glClearDepth( 1.0 );
        glEnable( GL_DEPTH_TEST );
        glDepthFunc( GL_LEQUAL );
        glHint( GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST );
        glPolygonMode( GL_BACK, GL_FILL );
        glPolygonMode( GL_FRONT, GL_FILL );
        isReaded=true;
    }
    sequence.open(videoPath);
    if (!sequence.isOpened())	{
        cerr << "Failed to open the image sequence!\n" << endl;
        exit(EXIT_FAILURE);
    }
    count=sequence.get(CAP_PROP_FRAME_COUNT);
    cur=0;
    first=true;
    int fps=sequence.get(CAP_PROP_FPS);
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(play()));
     timer->start((1000/fps));
     namedWindow("video",CV_WINDOW_AUTOSIZE|CV_GUI_EXPANDED|CV_WINDOW_KEEPRATIO);
}

void MeshViewerWidget::play()
{
    if(!models.empty()){
        MatF model=models.front();
        models.pop_front();
        MatF uv=uvs.front();
        uvs.pop_front();
        Mat faceTexture=faceTextures.front();
        faceTextures.pop_front();
        memcpy(pointData,model.data(),sizeof(float)*3*mesh.n_vertices());
        memcpy(uvData,uv.data(),sizeof(float)*2*mesh.n_vertices());
        QImage face=MatConvertQImage::Mat2QImage(faceTexture);
        if(textures[0]){
            delete textures[0];
        }
        textures[0] = new QOpenGLTexture(face);
        textures[0]->setMinificationFilter(QOpenGLTexture::Nearest);
        textures[0]->setMagnificationFilter(QOpenGLTexture::Linear);
        textures[0]->setWrapMode(QOpenGLTexture::Repeat);
        update();
        return;

    }
    if(cur<count-1){
        sequence >> image;
        if(image.empty()){
            std::cerr<<"read pic fail!"<<std::endl;
        }
        std::tie(models,uvs,faceTextures)=modelPtr->readOnePic(image,first,false);
        if(!models.empty()){
            MatF model=models.front();
            models.pop_front();
            MatF uv=uvs.front();
            uvs.pop_front();
            Mat faceTexture=faceTextures.front();
            faceTextures.pop_front();
            memcpy(pointData,model.data(),sizeof(float)*3*mesh.n_vertices());
            memcpy(uvData,uv.data(),sizeof(float)*2*mesh.n_vertices());
            QImage face=MatConvertQImage::Mat2QImage(faceTexture);
            if(textures[0]){
                delete textures[0];
            }
            textures[0] = new QOpenGLTexture(face);
            textures[0]->setMinificationFilter(QOpenGLTexture::Nearest);
            textures[0]->setMagnificationFilter(QOpenGLTexture::Linear);
            textures[0]->setWrapMode(QOpenGLTexture::Repeat);
            update();
            modelPtr->draw(faceTexture);
            cv::imshow("video",faceTexture);
        }
        cur++;
    }else{
        std::cout<<"stop:"<<std::endl;
        timer->stop();
        destroyWindow("video");
    }
}
void MeshViewerWidget::Clear(void)
{
    mesh.clear();
}

void MeshViewerWidget::UpdateMesh(void)
{
    mesh.update_normals();
    if (mesh.vertices_empty())
    {
        std::cerr << "ERROR: UpdateMesh() No vertices!" << std::endl;
        return;
    }
    ptMin[0] = ptMin[1] = ptMin[2] = DBL_MAX;
    ptMax[0] = ptMax[1] = ptMax[2] = -DBL_MAX;
    for (const auto& vh : mesh.vertices())
    {
        ptMin.minimize(mesh.point(vh));
        ptMax.maximize(mesh.point(vh));
    }

    double avelen = 0.0;
    double maxlen = 0.0;
    double minlen = DBL_MAX;
    for (const auto& eh : mesh.edges())
    {
        double len = mesh.calc_edge_length(eh);
        maxlen = len > maxlen ? len : maxlen;
        minlen = len < minlen ? len : minlen;
        avelen += len;
    }
    //    std::cout<<"MeshViewerWidget::UpdateMesh->SetScenePosition"<<std::endl;
    SetScenePosition((ptMin + ptMax)*0.5, (ptMin - ptMax).norm()*0.5);
    //    std::cout << "Information of the input mesh:" << std::endl;
    //    std::cout << "  [V, E, F] = [" << mesh.n_vertices() << ", " << mesh.n_edges() << ", " << mesh.n_faces() << "]\n";
    //    std::cout << "  BoundingBox:\n";
    //    std::cout << "  X: [" << ptMin[0] << ", " << ptMax[0] << "]\n";
    //    std::cout << "  Y: [" << ptMin[1] << ", " << ptMax[1] << "]\n";
    //    std::cout << "  Z: [" << ptMin[2] << ", " << ptMax[2] << "]\n";
    //    std::cout << "  Diag length of BBox: " << (ptMax - ptMin).norm() << std::endl;
    //    std::cout << "  Edge Length: [" << minlen << ", " << maxlen << "]; AVG: " << avelen / mesh.n_edges() << std::endl;
}

bool MeshViewerWidget::SaveMesh(const std::string & filename)
{
    return MeshTools::WriteMesh(mesh, filename, DBL_DECIMAL_DIG);
}

bool MeshViewerWidget::ScreenShot()
{
    update();
    QString filename = strMeshPath + "/" + QDateTime::currentDateTime().toString("yyyyMMddHHmmsszzz") + QString(".png");
    QImage image = grabFrameBuffer();
    image.save(filename);
    std::cout << "Save screen shot to " << filename.toStdString() << std::endl;
    return true;
}

void MeshViewerWidget::SetDrawBoundingBox(bool b)
{
    isDrawBoundingBox = b;
    update();
}
void MeshViewerWidget::SetDrawBoundary(bool b)
{
    isDrawBoundary = b;
    update();
}
void MeshViewerWidget::EnableLighting(bool b)
{
    isEnableLighting = b;
    update();
}
void MeshViewerWidget::EnableDoubleSide(bool b)
{
    isTwoSideLighting = b;
    update();
}

void MeshViewerWidget::ResetView(void)
{
    //std::cout<<"ResetView"<<std::endl;
    ResetModelviewMatrix();
    ViewCenter();
    update();
}

void MeshViewerWidget::ViewCenter(void)
{
    if (!mesh.vertices_empty())
    {
        UpdateMesh();
    }
    update();
}

void MeshViewerWidget::CopyRotation(void)
{
    CopyModelViewMatrix();
}

void MeshViewerWidget::LoadRotation(void)
{
    LoadCopyModelViewMatrix();
    update();
}

void MeshViewerWidget::PrintMeshInfo(void)
{
    std::cout << "Mesh Info:\n";
    std::cout << "  [V, E, F] = [" << mesh.n_vertices() << ", " << mesh.n_edges() << ", " << mesh.n_faces() << "]\n";
    std::cout << "  BoundingBox:\n";
    std::cout << "  X: [" << ptMin[0] << ", " << ptMax[0] << "]\n";
    std::cout << "  Y: [" << ptMin[1] << ", " << ptMax[1] << "]\n";
    std::cout << "  Z: [" << ptMin[2] << ", " << ptMax[2] << "]\n";
    std::cout << "  Diag length of BBox: " << (ptMax - ptMin).norm() << std::endl;
}


void MeshViewerWidget::DrawScene(void)
{
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixd(&projectionmatrix[0]);
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixd(&modelviewmatrix[0]);
    //DrawAxis();
    if (isDrawBoundingBox) DrawBoundingBox();
    if (isDrawBoundary) DrawBoundary();
    if (isEnableLighting) glEnable(GL_LIGHTING);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, isTwoSideLighting);
    DrawSceneMesh();
    if (isEnableLighting) glDisable(GL_LIGHTING);
}

void MeshViewerWidget::DrawSceneMesh(void)
{
    if (mesh.n_vertices() == 0) { return; }
    SetMaterial();
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    switch (drawmode)
    {
    case POINTS:
        DrawPoints();
        break;
    case WIREFRAME:
        DrawWireframe();
        break;
    case HIDDENLINES:
        DrawHiddenLines();
        break;
    case FLATLINES:
        DrawFlatLines();
        break;
    case FLAT:
        DrawFlat();
        break;
    case SMOOTH:
        DrawSmooth();
        break;
    default:
        break;
    }
}
void MeshViewerWidget::paintGL(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (meshQueue.size() > 0) {
        //        mesh = meshQueue.front(); meshQueue.pop_front();
        //        pic = imageQueue.front(); imageQueue.pop_front();
        //        face = faceTextureQueue.front(); faceTextureQueue.pop_front();
        //        head = headTextureQueue.front(); headTextureQueue.pop_front();
        //        if (!textures[0]) {
        //            /*delete textures[0];
        //            textures[0] = new QOpenGLTexture(MatConvertQImage::Mat2QImage(face).mirrored());
        //            textures[0]->setMagnificationFilter(QOpenGLTexture::Linear);*/
        //            QImage buf=MatConvertQImage::Mat2QImage(face).mirrored();
        //            QImage tex = QGLWidget::convertToGLFormat(buf);

        //            glGenTextures(1, &textures[0]);

        //            glBindTexture(GL_TEXTURE_2D, textures[0]);
        //            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        //            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        //            glTexImage2D(GL_TEXTURE_2D, 0, 3, tex.width(), tex.height(), 0,
        //                         GL_RGBA, GL_UNSIGNED_BYTE, tex.bits());
        //        }
        //        if (!textures[1]) {
        //            /*delete textures[1];
        //            textures[1] = new QOpenGLTexture(MatConvertQImage::Mat2QImage(head).mirrored());
        //            textures[1]->setMagnificationFilter(QOpenGLTexture::Linear);*/
        //            QImage buf = MatConvertQImage::Mat2QImage(face).mirrored();
        //            QImage tex = QGLWidget::convertToGLFormat(buf);

        //            glGenTextures(1, &textures[1]);

        //            glBindTexture(GL_TEXTURE_2D, textures[1]);
        //            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        //            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        //            glTexImage2D(GL_TEXTURE_2D, 0, 3, tex.width(), tex.height(), 0,
        //                         GL_RGBA, GL_UNSIGNED_BYTE, tex.bits());
        //        }

        //        if (!isUpdate) {
        //            UpdateMesh();
        //            isUpdate = true;
        //            cv::namedWindow("video", CV_WINDOW_NORMAL);
        //            cv::imshow("video", pic);
        //            //cv::waitKey(100);
        //        }
        //        else {
        //            mesh.update_normals();
        //            cv::imshow("video", pic);
        //            //cv::waitKey(100);
        //        }

    }
    else {
        /*if (timerId != -1) {
            killTimer(timerId);
            timerId = -1;
            isUpdate = false;
        }*/
        //cv::destroyWindow("video");
        isUpdate = false;

    }
    DrawScene();
}
void MeshViewerWidget::timerEvent(QTimerEvent *) {
    update();
}


void MeshViewerWidget::DrawPoints(void) const
{
    glColor3d(1.0, 0.5, 0.5);
    glPointSize(5);
    glBegin(GL_POINTS);
    for (const auto& vh : mesh.vertices())
    {
        glNormal3fv(mesh.normal(vh).data());
        glVertex3fv(mesh.point(vh).data());
    }
    glEnd();
}

void MeshViewerWidget::DrawWireframe(void) const
{
    glColor3d(0.2, 0.2, 0.2);
    glBegin(GL_LINES);
    for (const auto& eh : mesh.edges())
    {
        auto heh = mesh.halfedge_handle(eh, 0);
        auto vh0 = mesh.from_vertex_handle(heh);
        auto vh1 = mesh.to_vertex_handle(heh);
        glNormal3fv(mesh.normal(vh0).data());
        glVertex3fv(mesh.point(vh0).data());
        glNormal3fv(mesh.normal(vh1).data());
        glVertex3fv(mesh.point(vh1).data());
    }
    glEnd();
}

void MeshViewerWidget::DrawHiddenLines() const
{
    glLineWidth(1.0);
    float backcolor[4];
    glGetFloatv(GL_COLOR_CLEAR_VALUE, backcolor);
    glColor4fv(backcolor);
    glDepthRange(0.01, 1.0);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    if (glIsEnabled(GL_LIGHTING))
    {
        glDisable(GL_LIGHTING);
        DrawFlat();
        glEnable(GL_LIGHTING);
    }
    else
    {
        DrawFlat();
    }
    glDepthRange(0.0, 1.0);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glColor3d(.3, .3, .3);
    DrawFlat();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void MeshViewerWidget::DrawFlatLines(void) const
{
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.5f, 2.0f);
    glShadeModel(GL_FLAT);
    //glColor3d(0.8, 0.8, 0.8);
    glColor3d(1.0, 1.0, 1.0);
    DrawFlat();
    glDisable(GL_POLYGON_OFFSET_FILL);
    if (glIsEnabled(GL_LIGHTING))
    {
        glDisable(GL_LIGHTING);
        DrawWireframe();
        glEnable(GL_LIGHTING);
    }
    else
    {
        DrawWireframe();
    }
}

void MeshViewerWidget::DrawFlat(void) const
{
    if (textures[0] && textures[1] ) {
        Mesh::FaceIter f_it;
        textures[0]->bind();
        //glBindTexture(GL_TEXTURE_2D, textures[0]);
        glBegin(GL_TRIANGLES);

        for (f_it = mesh.faces_begin(); f_it != mesh.faces_begin()+headTriOffset; ++f_it)
        {
            glNormal3fv(mesh.normal(*f_it).data());
            for (const auto& fvh : mesh.fv_range(*f_it))
            {

                glTexCoord2fv(mesh.texcoord2D(fvh).data());
                glVertex3fv(mesh.point(fvh).data());
            }
        }
        glEnd();
        textures[0]->release();
        textures[1]->bind();
        //glBindTexture(GL_TEXTURE_2D, textures[1]);
        glBegin(GL_TRIANGLES);

        for (; f_it != mesh.faces_end(); ++f_it) {
            glNormal3fv(mesh.normal(*f_it).data());
            for (const auto& fvh : mesh.fv_range(*f_it))
            {
                glTexCoord2fv(mesh.texcoord2D(fvh).data());
                glVertex3fv(mesh.point(fvh).data());
            }
        }
        glEnd();
        textures[1]->release();
    }
    else {
        glBegin(GL_TRIANGLES);
        for (const auto& fh : mesh.faces())
        {
            glNormal3fv(mesh.normal(fh).data());
            for (const auto& fvh : mesh.fv_range(fh))
            {
                //glTexCoord2dv(mesh.texcoord2D(fvh).data());
                glVertex3fv(mesh.point(fvh).data());
            }
        }
        glEnd();
    }
    //textures[0]->bind();


}

void MeshViewerWidget::DrawSmooth(void) const
{
    glColor3d(0.8, 0.8, 0.8);
    //textures[0]->bind();
    glShadeModel(GL_SMOOTH);
    glLoadName(static_cast<GLuint>(mesh.n_vertices()));
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3, GL_DOUBLE, 0, mesh.points());
    glEnableClientState(GL_NORMAL_ARRAY);
    glNormalPointer(GL_DOUBLE, 0, mesh.vertex_normals());
    for (const auto& fh : mesh.faces())
    {
        glBegin(GL_POLYGON);
        for (const auto& fvh : mesh.fv_range(fh))
        {
            glArrayElement(fvh.idx());
        }
        glEnd();
    }
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
}

void MeshViewerWidget::DrawBoundingBox(void) const
{
    float linewidth;
    glGetFloatv(GL_LINE_WIDTH, &linewidth);
    glLineWidth(2.0f);
    glColor3d(.3, .7, .3);
    glBegin(GL_LINES);
    for (const auto& i : { 0, 1 })
    {
        for (const auto& j : { 0, 1 })
        {
            for (const auto& k : { 0, 1 })
            {
                glVertex3d(i ? ptMin[0] : ptMax[0], j ? ptMin[1] : ptMax[1], k ? ptMin[2] : ptMax[2]);
                glVertex3d(~i ? ptMin[0] : ptMax[0], j ? ptMin[1] : ptMax[1], k ? ptMin[2] : ptMax[2]);
                glVertex3d(i ? ptMin[0] : ptMax[0], j ? ptMin[1] : ptMax[1], k ? ptMin[2] : ptMax[2]);
                glVertex3d(i ? ptMin[0] : ptMax[0], ~j ? ptMin[1] : ptMax[1], k ? ptMin[2] : ptMax[2]);
                glVertex3d(i ? ptMin[0] : ptMax[0], j ? ptMin[1] : ptMax[1], k ? ptMin[2] : ptMax[2]);
                glVertex3d(i ? ptMin[0] : ptMax[0], j ? ptMin[1] : ptMax[1], ~k ? ptMin[2] : ptMax[2]);
            }
        }
    }
    glEnd();
    glLineWidth(linewidth);
}

void MeshViewerWidget::DrawBoundary(void) const
{
    float linewidth;
    glGetFloatv(GL_LINE_WIDTH, &linewidth);
    glLineWidth(2.0f);
    glColor3d(0.1, 0.1, 0.1);
    glBegin(GL_LINES);
    for (const auto& eh : mesh.edges())
    {
        if (mesh.is_boundary(eh))
        {
            auto heh = mesh.halfedge_handle(eh, 0);
            auto vh0 = mesh.from_vertex_handle(heh);
            auto vh1 = mesh.to_vertex_handle(heh);
            glNormal3fv(mesh.normal(vh0).data());
            glVertex3fv(mesh.point(vh0).data());
            glNormal3fv(mesh.normal(vh1).data());
            glVertex3fv(mesh.point(vh1).data());
        }
    }
    glEnd();
    glLineWidth(linewidth);
}
