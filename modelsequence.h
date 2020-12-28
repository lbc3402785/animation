#ifndef MODELSEQUENCE_H
#define MODELSEQUENCE_H
#include <string>
#include <queue>
#include <opencv2/core.hpp>
#include <QTime>
#include "../src/MMSolver.h"
#include "denselaplace.h"
class ModelSequence
{
private:
    QTime time;
    std::shared_ptr<DenseLaplace<float>> laplaceSolverPtr;

    double focalLengthmm=4.5;
    double sensorWidth=2.76;
    int success=0;
//    float lambdaSX[4]={40,30,25,19.39f};
//    float lambdaEX[4]={8,6,5,2.69f};
    float lambdaSX[2]={50,25.0f};
    float lambdaEX[2]={4.0,0.40f};
//    float lambdaEX[4]={0.1,0.05,0.02,0.01f};
    VectorXf rsv;
    VectorXf rev;
    cv::Mat Pymsk;  
    std::vector<int> keys;
    std::vector<int> face2BackHeadBds;
    std::vector<int> backHead2FaceBds;
    std::vector<int> neckBds;
    int headOffset;
    std::vector<float> vertices;
    std::vector<float> target;
    std::vector<int> faces;
    std::vector<int> roi;
    Quaternionf v0;
    Quaternionf v1;
    float lastDelta;
    float lastZMove;
    std::string outfolder="output\\";
    bool KeypointDetectgion(Mat& image, MatF &KP);
    void InitImages(std::string mskimage, std::string face,std::string head,std::string body);
    void InitModel(std::string npzPath,std::string dlibPath,std::string keyFilePath);

    MatF KP;
    MatF lastKP;
    MatF shapeKey3D;
    MatF expressKey3D;
    MatF lastSX;
    MatF lastEX;
    Mat lastImage;
    Eigen::Vector3f firstLoc;
    bool generateModel(cv::Mat& input,std::string name,bool save=true);
    MatF computeKey3D(MatF& lastKey3D,MatF& lastKey2D,MatF& KP,float fx);
    int H;
    int W;
    std::deque<std::tuple<MatF,MatF,cv::Mat>> results;
public:
    void draw(cv::Mat& input);
    bool center=true;
    cv::Mat FaceTexture;
    cv::Mat HeadTexture;
    cv::Mat BodyTexture;
    MMSolver solver;
    MatF generatedUV;

    ModelSequence();
    bool gerenateFromOnePic(std::string picPath);
    bool Init(std::string npzPath,std::string dlibPath,std::string facePath,std::string headPath,string bodyPath,std::string maskPath,std::string keyFilePath);
    std::tuple<MatF,MatF,cv::Mat> output(cv::Mat&input,std::string name,bool first=false,bool save=true);
    void readVideo(std::string videoPath);
    std::deque<std::tuple<MatF,MatF,cv::Mat>>& readOnePic(cv::Mat& input,bool& first,bool save=false);
    int getH() const;
    void setH(int value);
    int getW() const;
    void setW(int value);
};

#endif // MODELSEQUENCE_H
