#ifndef MODELSEQUENCE_H
#define MODELSEQUENCE_H
#include <string>
#include <queue>
#include <opencv2/core.hpp>
#include "../src/MMSolver.h"
#include "denselaplace.h"
class ModelSequence
{
private:
    std::shared_ptr<DenseLaplace<float>> laplaceSolverPtr;

    double focalLengthmm=4.5;
    double sensorWidth=2.76;
    int success=0;
    float lambda=0.008f;
    VectorXf rv;
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
    std::string outfolder="output\\";
    bool KeypointDetectgion(Mat image, MatF &KP);
    void InitImages(std::string mskimage, std::string face,std::string head,std::string body);
    void InitModel(std::string npzPath,std::string dlibPath,std::string keyFilePath);

    MatF KP;
    MatF lastKP;
    MatF shapeKey3D;
    MatF lastEX;
    Mat lastImage;
    bool generateModel(cv::Mat& input,std::string name,bool save=true);
    MatF computeKey3D(MatF& lastKey3D,MatF& lastKey2D,MatF& KP,float fx);
public:
    void draw(cv::Mat& input);
    bool center=true;
    cv::Mat FaceTexture;
    cv::Mat HeadTexture;
    cv::Mat BodyTexture;
    MMSolver solver;
    int H=1024;
    int W=1024;
    ModelSequence();
    bool gerenateFromOnePic(std::string picPath);
    bool Init(std::string npzPath,std::string dlibPath,std::string facePath,std::string headPath,string bodyPath,std::string maskPath,std::string keyFilePath);
    std::tuple<MatF,MatF,cv::Mat> output(cv::Mat&input,std::string name,bool save=true);
    void readVideo(std::string videoPath);
    std::tuple<std::deque<MatF>,std::deque<MatF>,std::deque<cv::Mat>> readOnePic(cv::Mat& input,bool& first,bool save=false);
};

#endif // MODELSEQUENCE_H
