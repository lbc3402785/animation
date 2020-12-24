#include "modelsequence.h"
#include "src/preprocess.h"
#define _USE_MATH_DEFINES   //　看math.h中的定义
#include <math.h>
#include "src/Dlib.h"
#include "src/Poisson.h"
#include "common/imageprocess.h"
#include "process/pointcloudregister.h"
#include "meshtools.h"
void ModelSequence::InitImages(string mskimage, string face, string head, string body)
{
    Mat msk0 = imread(mskimage, 0);
    Pymsk = (msk0 == 255);//人脸区域mask
    FaceTexture = imread(face);//人脸模板图
    HeadTexture=imread(head);//头部模板图
    //BodyTexture=imread(body);//身体模板图
    if(FaceTexture.size!=HeadTexture.size||FaceTexture.size!=Pymsk.size||FaceTexture.rows!=H||FaceTexture.cols!=W){
        std::cerr<<"size is not match"<<std::endl;
        exit(EXIT_FAILURE);
    }
}

void ModelSequence::InitModel(string npzPath, string dlibPath, string keyFilePath)
{
    DlibInit(dlibPath);
    FaceModel bfmShape;
    bfmShape.Initialize(npzPath,true,false);
    Preprocess::loadKeyIndex(keyFilePath,keys,true);
//    {
//        Eigen::Matrix3Xf BFMpoints=bfmShape.Face.transpose();
//        Eigen::Matrix3Xi BFMFaces=bfmShape.TRI.transpose();
//        EigenFunctions<float>::saveEigenPointsWithKey(BFMpoints,BFMFaces,keys,"BFMkeys.obj");
//    }
    //std::cout<<"HEAD_OFFSET:"<<bfmShape.HEAD_OFFSET<<std::endl;
    //std::cout<<"TRI_FACE.rows():"<<bfmShape.TRI_FACE.rows()<<std::endl;
    FaceModel bfmKeyShape;
    Preprocess::extractBFMKeyModel(bfmShape,keys,bfmKeyShape);
    solver.FMFull=bfmShape;
    solver.FM=bfmKeyShape;
    vertices.resize(bfmShape.Face.size());
    memcpy(vertices.data(),bfmShape.Face.data(),sizeof(float)*bfmShape.Face.size());
    faces.resize(bfmShape.TRI.size());
    memcpy(faces.data(),bfmShape.TRI.data(),sizeof(int)*bfmShape.TRI.size());
    face2BackHeadBds=MeshTools::getBoundaries(vertices,faces,29846);
    //std::cout<<"face2BackHeadBds done!"<<std::endl;
    backHead2FaceBds=MeshTools::getBoundaries(vertices,faces,47583);
    //std::cout<<"backHead2FaceBds done!"<<std::endl;
    neckBds=MeshTools::getBoundaries(vertices,faces,46399);
    //std::cout<<"neckBds done!"<<std::endl;
    Preprocess::extractKeyFace(vertices,neckBds,target);
    headOffset=bfmShape.HEAD_OFFSET(0,0);
    MatF faceBdV;
    Preprocess::extractKeyFace(bfmShape.Face,face2BackHeadBds,faceBdV);
    float minY=faceBdV.col(1).minCoeff();
    float maxY=faceBdV.col(1).maxCoeff();
    float eps=0.5*minY+0.5*maxY;
    std::map<int,int> tmp;
    for(int k=headOffset;k<bfmShape.Face.rows();k++){
        if(bfmShape.Face(k,1)<eps){
            roi.emplace_back(k);
            tmp[k]=k;
        }
    }
    for(int k=0;k<bfmShape.TRI_HEAD.rows();k++){
        int i0=bfmShape.TRI_HEAD(k,0);
        int i1=bfmShape.TRI_HEAD(k,1);
        int i2=bfmShape.TRI_HEAD(k,2);
        if(tmp.count(i0)>0||tmp.count(i1)>0||tmp.count(i2)>0){
            roi.emplace_back(i0);
            roi.emplace_back(i1);
            roi.emplace_back(i2);
        }
    }
    std::sort(roi.begin(), roi.end());
    vector<int>::iterator ip=std::unique(roi.begin(), roi.end());
    roi.resize(std::distance(roi.begin(), ip));
    //EigenFunctions<float>::saveEigenPointsWithKey(vertices,faces,roi,"roi.obj");
    std::vector<int> fixed=SetOperator<int>::intersection(roi,backHead2FaceBds);
    DenseLaplace<float> denseSolver(vertices,faces,neckBds,fixed);
    //std::cout<<"init done!"<<std::endl;
    denseSolver.setRoi(roi);
    //std::cout<<"setRoi done!"<<std::endl;
    laplaceSolverPtr=std::make_shared<DenseLaplace<float>>(denseSolver);
}

MatF ModelSequence::computeKey3D(MatF &lastKey3D, MatF &lastKey2D, MatF &KP,float fx)
{
    MatF result=lastKey3D;
    MatF diff2D=KP-lastKey2D;
    //std::cout<<"diff2D:"<<diff2D<<std::endl;
    Eigen::RowVectorXf p1t=lastKey3D.col(0);
    Eigen::RowVectorXf p2t=lastKey3D.col(1);
    Eigen::RowVectorXf p3t=lastKey3D.col(2);
    Eigen::RowVectorXf I1t=diff2D.col(0);
    Eigen::RowVectorXf I2t=diff2D.col(1);
    //std::cout<<"fx:"<<fx<<std::endl;
    //std::cout<<"p3t.cwiseProduct(I1t):"<<p3t.cwiseProduct(I1t)<<std::endl;
    //std::cout<<"p3t.cwiseProduct(I2t):"<<p3t.cwiseProduct(I2t)<<std::endl;
    Eigen::RowVectorXf q1t=p3t.cwiseProduct(I1t)/fx+p1t;
    Eigen::RowVectorXf q2t=p3t.cwiseProduct(I2t)/fx+p2t;
    result.col(0)=q1t;
    result.col(1)=q2t;
    return result;
}

void ModelSequence::draw(Mat &input)
{
    MatF projected=KP;
    if(center){
        projected.col(0).array()+=input.cols/2;
        projected.col(1).array()+=input.rows/2;
    }
    for (size_t i = 0; i < projected.rows(); i++)
    {
        auto x = projected(i, 0);
        auto y = projected(i, 1);
        circle(input, Point(x, y), 8, Scalar(255, 0, 0, 255), -1, CV_AA);
    }
}

ModelSequence::ModelSequence()
    :v0(1,0,0,0),v1(1,0,0,0)
{
}

bool ModelSequence::gerenateFromOnePic(string picPath)
{
    cv::Mat input=cv::imread(picPath);

    if(KeypointDetectgion(ImageProcess::LaplacianEnhance(input), KP)){
        cv::Mat show=input.clone();
        for (size_t i = 0; i < KP.rows(); i++)
        {
            auto x = KP(i, 0);
            auto y = KP(i, 1);
            circle(show, Point(x, y), 8, Scalar(255, 0, 0, 255), -1, CV_AA);
        }
        cv::imshow("11",show);
        cv::waitKey(0);
        if(center){
            KP.col(0).array()-=input.cols/2;
            KP.col(1).array()-=input.rows/2;
        }
        double fx=focalLengthmm*static_cast<double>(input.cols)/sensorWidth;
        solver.fx=fx;
        generateModel(input,"50");
        return true;
    }
    return false;
}

bool ModelSequence::Init(string npzPath, string dlibPath, string facePath, string headPath,string bodyPath, string maskPath,string keyFilePath)
{
    InitImages(maskPath,facePath,headPath,bodyPath);
    InitModel(npzPath,dlibPath,keyFilePath);
    MatF EV=solver.FMFull.EV;
    int n=EV.rows();
    rv=VectorXf::Ones(n);
    for(int k=0;k<n;k++){
        rv(k)=1/EV(k,0);
    }
    rv.normalize();
    rv*=n*lambda;
    return true;
}
bool ModelSequence::KeypointDetectgion(Mat image, MatF &KP)
{
    if(image.empty())return false;
    vector<vector<Point>> keypoints;
    double S = 0.5;
    Mat simage;
    cv::resize(image, simage, Size(), S, S);

    vector<Rect> rectangles;
    DlibFace(simage, rectangles, keypoints);

    if (keypoints.size() <= 0)
    {
        errorcout << "NO POINTS" << endl;
        return false;
    }

    KP = ToEigen(keypoints[0]) * (1.0 / S);

    return true;
}
bool ModelSequence::generateModel(Mat &image,std::string name,bool save)
{
    solver.params.R=MatF::Identity(3,3);
    solver.params.tx=100;
    solver.params.ty=100;
    solver.params.ty=500;
    solver.SolvePerspective2(KP);
    if(save){
        output(image,name);
    }
    return true;
}

std::tuple<MatF,MatF,cv::Mat> ModelSequence::output(Mat &image, string name,bool save)
{
    MatF model;
    MatF UV;
    Mat texture;

    auto Face = solver.FMFull.Generate(solver.SX, solver.EX);
    MatF rotateFace=Face*solver.params.R.transpose();
    rotateFace.col(1).array()*=-1;
    rotateFace.col(2).array()*=-1;
    laplaceSolverPtr->updatePoints(rotateFace.data(),rotateFace.size());
    laplaceSolverPtr->solve(target);
    rotateFace=laplaceSolverPtr->getDst();
    if(0){
        Mat origin=MMSPerspectiveTexture(image,solver,H,W,center);
        Scalar diff;
        //result = PoissonBlending(texture, FaceTexture, Pymsk, true,30,30,&diff);
        if(save){
            cv::imwrite(outfolder+name + "Face.png",origin );
            cv::imwrite(outfolder+name+"Head.png",HeadTexture+diff*255);
            //MMSObjWithTexture(result*255, solver, outfolder, name);
            MMSObj2(rotateFace,solver,outfolder,name);
        }
        texture=origin;
    }else{
        texture=image;
        MatF frontFace=Face.topRows(headOffset);
        MatF frontFacePro = solver.PerspectiveProjection(solver.params, frontFace);
        if(center){
            frontFacePro.col(0).array()+=image.cols/2;
            frontFacePro.col(1).array()+=image.rows/2;
        }
        frontFacePro.col(0)/=image.cols;
        frontFacePro.col(1)/=image.rows;
        MatF mergeUV=solver.FMFull.UV;
        mergeUV.topRows(headOffset)=frontFacePro;
        UV=mergeUV;
        if(save){
            cv::imwrite(outfolder+name + "Face.png",image );
            cv::imwrite(outfolder+name+"Head.png",HeadTexture);
            Eigen::Matrix3Xf points=rotateFace.transpose();
            Eigen::Matrix3Xi faces=solver.FMFull.TRI.transpose();
            Eigen::Matrix2Xf uvs=mergeUV.transpose();
            EigenFunctions<float>::saveEigenPointsWithUV(points,faces,uvs,"output\\"+name+".obj");
        }
    }
    model=rotateFace;

    return std::make_tuple(model,UV,texture);
}

void ModelSequence::readVideo(string videoPath)
{
    using namespace std;
    using namespace cv;
    VideoCapture sequence(videoPath);
    if (!sequence.isOpened())	{
        cerr << "Failed to open the image sequence!\n" << endl;
        exit(EXIT_FAILURE);
    }
    int count=sequence.get(CAP_PROP_FRAME_COUNT);
    cv::Mat image;
    bool first=true;
    for(int k=0;k<count;k++){
        sequence >> image;
        readOnePic(image,first,true);
    }
}

std::tuple<std::deque<MatF>,std::deque<MatF>,std::deque<cv::Mat>> ModelSequence::readOnePic(Mat& image,bool& first,bool save)
{
    using namespace std;
    using namespace cv;
    std::deque<MatF> models;
    std::deque<MatF> uvs;
    std::deque<Mat> textures;
    MatF model;
    MatF uv;
    Mat texture;
    if(first){
        if(KeypointDetectgion(ImageProcess::LaplacianEnhance(image), KP)){
            if(center){
                KP.col(0).array()-=image.cols/2;
                KP.col(1).array()-=image.rows/2;
            }

            double fx=focalLengthmm*static_cast<double>(image.cols)/sensorWidth;
            solver.fx=fx;
            solver.params.R=MatF::Identity(3,3);
            solver.params.tx=100;
            solver.params.ty=100;
            solver.params.ty=500;
            solver.SolvePerspective2(KP);
            solver.FM.Generate(solver.SX, solver.EX*0);
            shapeKey3D=solver.Transform(solver.params,solver.FM.GeneratedFace);
            solver.FMFull.Generate(solver.SX, solver.EX);
            solver.FM.Generate(solver.SX, solver.EX);
            lastKP=KP;
            lastImage=image;
            lastEX=solver.EX;
            v0=solver.params.R;
            first=false;
            std::tie(model,uv,texture)=output(image, std::to_string(success),save);
            models.emplace_back(model);
            uvs.emplace_back(uv);
            textures.emplace_back(texture);
            success++;
        }
    }else{
        if(KeypointDetectgion(image, KP)){

            if(center){
                KP.col(0).array()-=image.cols/2;
                KP.col(1).array()-=image.rows/2;
            }
            solver.FM.Generate(solver.SX, solver.EX);
            MatF lastKey3D=solver.Transform(solver.params,solver.FM.GeneratedFace);
            MatF lastKey2D=solver.PerspectiveProjection(solver.params,solver.FM.GeneratedFace);
            MatF key3D=computeKey3D(lastKey3D,lastKey2D/*lastKP*/,KP,solver.fx);
            Eigen::Matrix3Xf srcKeyPoints=lastKey3D.transpose();
            Eigen::Matrix3Xf dstKeyPoints=key3D.transpose();
            Rigid::Transform<float> initTransform;
            Rigid::PointCloudRegister<float>::registerNPoint(srcKeyPoints,dstKeyPoints,initTransform);
            Eigen::Matrix3f R0=solver.params.R;
            Eigen::Vector3f t0(solver.params.tx,solver.params.ty,solver.params.tz);
            Eigen::Matrix3f R1=initTransform.getRotation()*R0;
            Eigen::Vector3f t1=initTransform.getRotation()*t0+initTransform.getTranslation();
            solver.params.R=R1;
            solver.params.tx=t1(0);
            solver.params.ty=t1(1);
            solver.params.tz=t1(2);
            //                MatF EX=solver.SolveShape2(solver.params,shapeKey3D,key3D,solver.FM.EB,0.008f);
            MatF EX=solver.SolveShape3(solver.params,shapeKey3D,key3D,solver.FM.EB,rv);
            v1=R1;
            Quaternionf v01=v0.slerp(0.3,v1);
            Eigen::Matrix3f R01=v01.toRotationMatrix();
            Eigen::Vector3f t01=0.6*t0+0.4*t1;
            solver.params.R=R01;
            solver.params.tx=t01(0);
            solver.params.ty=t01(1);
            solver.params.tz=t01(2);
            if(success>=2){
                solver.EX=(lastEX*0.3+solver.EX*0.3+EX*0.4);
            }else if(success>=1){
                solver.EX=(solver.EX*0.6+EX*0.4);
            }
            cv::Mat interImage=0.6*lastImage+0.4*image;
            std::tie(model,uv,texture)=output(interImage,std::to_string(success),save);
            models.emplace_back(model);
            uvs.emplace_back(uv);
            textures.emplace_back(texture);
            lastEX=solver.EX;

            //===================================================
            Quaternionf v02=v0.slerp(0.6,v1);
            Eigen::Matrix3f R02=v02.toRotationMatrix();
            Eigen::Vector3f t02=0.4*t0+0.6*t1;
            solver.params.R=R02;
            solver.params.tx=t02(0);
            solver.params.ty=t02(1);
            solver.params.tz=t02(2);
            if(success>=2){
                solver.EX=(lastEX*0.2+solver.EX*0.2+EX*0.6);
            }else if(success>=1){
                solver.EX=(solver.EX*0.4+EX*0.6);
            }
            cv::Mat interImage2=0.4*lastImage+0.6*image;
            std::tie(model,uv,texture)=output(interImage2,std::to_string(success+1),save);
            models.emplace_back(model);
            uvs.emplace_back(uv);
            textures.emplace_back(texture);
            lastEX=solver.EX;
            //solver.params.R=EigenFunctions<float>::rodrigues(v2);
            solver.params.R=R1;
            solver.params.tx=t1(0);
            solver.params.ty=t1(1);
            solver.params.tz=t1(2);
            solver.EX=EX;
            std::tie(model,uv,texture)=output(image,std::to_string(success+2),save);
            models.emplace_back(model);
            uvs.emplace_back(uv);
            textures.emplace_back(texture);
            lastKP=KP;
            lastImage=image;
            v0=v1;
            success+=3;
        }

    }
    return std::make_tuple(models,uvs,textures);
}
