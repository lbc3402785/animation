#define  _USE_MATH_DEFINES
#include "imageprocess.h"
#include <iostream>
#include <cmath>
using namespace std;
using namespace cv;
ImageProcess::ImageProcess()
{

}
cv::Mat ImageProcess::LaplacianEnhance(cv::Mat& input){
    Mat h1_kernel = (Mat_<char>(3, 3) << -1, -1, -1, -1, 8, -1, -1, -1, -1);
    Mat h2_kernel = (Mat_<char>(3, 3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);

    Mat h1_result,h2_result;
    filter2D(input, h1_result, CV_32F, h1_kernel);
    filter2D(input, h2_result, CV_32F, h2_kernel);
    convertScaleAbs(h1_result, h1_result);
    convertScaleAbs(h2_result, h2_result);
    return h2_result;
}
void ImageProcess::GammaTransform(const Mat& srcImage,Mat& dstImage,double gamma)
{
    int type=srcImage.type();
    uchar depth = type & CV_MAT_DEPTH_MASK;
    assert(depth==CV_8U);
    int channels = srcImage.channels();
    unsigned char lut[256];
    for(int i=0;i < 256;i++)
    {
        lut[i] = saturate_cast<uchar>(pow((float)i/255.0,gamma) * 255.0f);
    }

    dstImage = srcImage.clone();

    switch(channels)
    {
    case 1:
    {
        MatIterator_<uchar> it = dstImage.begin<uchar>();
        MatIterator_<uchar> end = dstImage.end<uchar>();
        while(it != end)
        {
            *it = lut[(*it)];
            it ++;
        }
        break;
    }
    case 3:
    {
        MatIterator_<Vec3b> it = dstImage.begin<Vec3b>();
        MatIterator_<Vec3b> end = dstImage.end<Vec3b>();
        while(it != end)
        {
            (*it)[0] = lut[(*it)[0]];
            (*it)[1] = lut[(*it)[1]];
            (*it)[2] = lut[(*it)[2]];
            it ++;
        }
        break;
    }
    default:
        break;
    }
}

void ImageProcess::LightCompensation(const Mat &srcImage, Mat &dstImage, double pencent)
{
    cv::Mat in=srcImage;
    float gamma=2.2f;
    ImageProcess::GammaTransform(srcImage,in,gamma);
    int channels = srcImage.channels();
    switch(channels)
    {
    case 1:
    {
        unsigned char top=0;
        unsigned char threshold=0;
        MatIterator_<uchar> it = in.begin<uchar>();
        MatIterator_<uchar> end = in.end<uchar>();
        int sum=0;
        int count=0;
        while(it != end)
        {
            if(*it>top){
                top=*it;
            }
            it ++;
        }
        it = in.begin<uchar>();
        threshold=saturate_cast<uchar>(top*(1.0f-pencent));
        while(it != end)
        {
            if(*it>threshold){
                sum+=saturate_cast<int>(*it);
                count++;
            }
            it ++;
        }
        int avg=saturate_cast<int>(sum/count);
        it = in.begin<uchar>();
        while(it != end)
        {
            if(*it>threshold){
                *it=255;
            }else{
                *it=*it*255/avg;
            }
            it ++;
        }
        break;
    }

    case 3:{
        unsigned char tops[3]={0,0,0};
        unsigned char thresholds[3]={0,0,0};
        int sums[3]={0,0,0};
        int counts[3]={0,0,0};
        MatIterator_<Vec3b> it = in.begin<Vec3b>();
        MatIterator_<Vec3b> end = in.end<Vec3b>();
        while(it != end)
        {
            if((*it)[0]>tops[0]){
                tops[0]=(*it)[0];
            }
            if((*it)[1]>tops[1]){
                tops[1]=(*it)[1];
            }
            if((*it)[2]>tops[2]){
                tops[2]=(*it)[2];
            }
            it ++;
        }
        it = in.begin<Vec3b>();
        thresholds[0]=saturate_cast<uchar>(tops[0]*(1.0f-pencent));
        thresholds[1]=saturate_cast<uchar>(tops[1]*(1.0f-pencent));
        thresholds[2]=saturate_cast<uchar>(tops[2]*(1.0f-pencent));
        while(it != end)
        {
            if((*it)[0]>thresholds[0]){
                sums[0]+=saturate_cast<int>((*it)[0]);
                counts[0]++;
            }
            if((*it)[1]>thresholds[1]){
                sums[1]+=saturate_cast<int>((*it)[1]);
                counts[1]++;
            }
            if((*it)[2]>thresholds[0]){
                sums[2]+=saturate_cast<int>((*it)[2]);
                counts[2]++;
            }
            it ++;
        }
        int avgs[3]={0,0,0};
        avgs[0]=sums[0]/counts[0];
        avgs[1]=sums[1]/counts[1];
        avgs[2]=sums[2]/counts[2];
        it = in.begin<Vec3b>();
        while(it != end)
        {
            if((*it)[0]>thresholds[0]){
                (*it)[0]=255;
            }else{
                (*it)[0]=(*it)[0]*255/avgs[0];
            }
            if((*it)[1]>thresholds[1]){
                (*it)[1]=255;
            }else{
                (*it)[1]=(*it)[1]*255/avgs[1];
            }
            if((*it)[2]>thresholds[0]){
                (*it)[2]=255;
            }else{
                (*it)[2]=(*it)[2]*255/avgs[2];
            }
            it ++;
        }
        break;
    }
    }
    ImageProcess::GammaTransform(in,dstImage,1.0f/gamma);
}
void ImageProcess::BGR2HSI(const Mat& src, Mat& channelH, Mat& channelS, Mat& channelI){
    Mat matBGR[3];
    split(src, matBGR);
    Mat channelB, channelG, channelR;
    matBGR[0].convertTo(channelB, CV_32FC1);
    matBGR[1].convertTo(channelG, CV_32FC1);
    matBGR[2].convertTo(channelR, CV_32FC1);

    Mat matMin, matSum;
    add(channelB, channelG, matSum); // R G B 之和
    add(matSum, channelR, matSum);
    divide(channelB, matSum, channelB);  // 求解 b g r
    divide(channelG, matSum, channelG);
    divide(channelR, matSum, channelR);

    // 计算饱和度 s
    channelS.create(src.rows, src.cols, CV_32FC1);
    min(channelB, channelG, matMin);
    min(matMin, channelR, matMin);
    subtract(Mat(src.rows, src.cols, CV_32FC1, Scalar(1)), matMin * 3, channelS);

    // 计算 h
    channelH.create(src.rows, src.cols, CV_32FC1);
    float* bData = channelB.ptr<float>(0);
    float* gData = channelG.ptr<float>(0);
    float* rData = channelR.ptr<float>(0);
    float* hData = channelH.ptr<float>(0);
    float r, g, b, temp;
    for (int i = 0; i < src.rows * src.cols; i++){
        b = bData[i]; g = gData[i]; r = rData[i];

        // 单独处理 灰度图像
        if (b == g && b == r){
            hData[i] = 0.0f;
            continue;
        }

        temp = 0.5 * ((r - g)+(r - b))/sqrt((r - g)*(r - g) +(r - b)*(g - b));
        if (b <= g){
            hData[i] = acos(temp);
        }else{
            hData[i] = 2*M_PI - acos(temp);
        }
    }

    // 计算强度 I
    divide(matSum, 3 , channelI);
}
// h s I 转换到 BGR
void ImageProcess::HSI2BGR(Mat& channelH, Mat& channelS, Mat& channelI, Mat& dst){
    Mat bgr[3];
    bgr[0].create(channelH.size(), CV_8UC1);
    bgr[1].create(channelH.size(), CV_8UC1);
    bgr[2].create(channelH.size(), CV_8UC1);

    float* hData = channelH.ptr<float>(0);
    float* sData = channelS.ptr<float>(0);
    float* iData = channelI.ptr<float>(0);
    uchar* bData = bgr[0].ptr<uchar>(0);
    uchar* gData = bgr[1].ptr<uchar>(0);
    uchar* rData = bgr[2].ptr<uchar>(0);

    float h, s, i, x, y, z;
    for (int k = 0; k < channelH.rows * channelH.cols; k++){
        h = hData[k]; s = sData[k]; i = iData[k];
        if (h < 2*M_PI/3){
            x = i * (1 - s);
            y = i * (1 + s*cos(h)/cos(M_PI/3-h));
            z = 3 * i - (x + y);

            bData[k] = (uchar)x;
            gData[k] = (uchar)z;
            rData[k] = (uchar)y;
        }else if (h < 4*M_PI/3){
            h = h - 2*M_PI/3;
            x = i * (1 - s);
            y = i * (1 + s*cos(h)/cos(M_PI/3-h));
            z = 3 * i - (x + y);

            bData[k] = (uchar)z;
            gData[k] = (uchar)y;
            rData[k] = (uchar)x;
        }else if (h <= 2*M_PI){
            h = h - 4*M_PI/3;
            x = i * (1 - s);
            y = i * (1 + s*cos(h)/cos(M_PI/3-h));
            z = 3 * i - (x + y);

            bData[k] = (uchar)y;
            gData[k] = (uchar)x;
            rData[k] = (uchar)z;
        }
    }

    merge(bgr,3, dst);
}

void ImageProcess::GrayWorldColorBalance(const Mat &srcImage, Mat &dstImage)
{
    using namespace cv;
    cv::Mat in;
    float gamma=2.2f;
    ImageProcess::GammaTransform(srcImage,in,gamma);
    int channels = srcImage.channels();
    assert(channels==3);
    Mat inf;
    in.convertTo(inf, CV_32FC3);
cv:Scalar meanVal = cv::mean( inf );
    float mean=(meanVal[0]+meanVal[1]+meanVal[2])/3;
    Mat matBGR[3];
    split(inf, matBGR);
    matBGR[0]*=mean/meanVal[0];
    matBGR[1]*=mean/meanVal[1];
    matBGR[2]*=mean/meanVal[2];
    Mat dstImagef;
    merge(matBGR,3,dstImagef);
    dstImagef.convertTo(dstImage,CV_8UC3);
    ImageProcess::GammaTransform(in,dstImage,1.0f/gamma);
}
void ImageProcess::inpaintImage(const Mat &src, Mat &dst)
{
    Mat imageGray;
    //转换为灰度图
    cvtColor(src, imageGray, CV_RGB2GRAY, 0);
    Mat imageMask = Mat(src.size(), CV_8UC1, Scalar::all(0));

    //通过阈值处理生成Mask
    threshold(imageGray, imageMask, 240, 255, CV_THRESH_BINARY);
    Mat Kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    //对Mask膨胀处理，增加Mask面积
    dilate(imageMask, imageMask, Kernel);

    //图像修复
    inpaint(src, imageMask, dst, 5, INPAINT_TELEA);
    waitKey();
}
