#ifndef IMAGEPROCESS_H
#define IMAGEPROCESS_H

#include <opencv2/opencv.hpp>
class ImageProcess
{
public:
    ImageProcess();
    static cv::Mat LaplacianEnhance(cv::Mat& input);
    static void GammaTransform(const cv::Mat& srcImage,cv::Mat& dstImage,double gamma);
    static void LightCompensation(const cv::Mat& srcImage,cv::Mat& dstImage,double pencent=0.05);
    static void BGR2HSI(const cv::Mat& src, cv::Mat& channelH, cv::Mat& channelS, cv::Mat& channelI);
    static void HSI2BGR(cv::Mat& channelH, cv::Mat& channelS, cv::Mat& channelI, cv::Mat& dst);
    static void GrayWorldColorBalance(const cv::Mat& srcImage,cv::Mat& dstImage);
    static void inpaintImage(const cv::Mat&src,cv::Mat&dst);
};

#endif // IMAGEPROCESS_H
