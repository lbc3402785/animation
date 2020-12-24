#ifndef MATCONVERTQIMAGE_H
#define MATCONVERTQIMAGE_H
#include <opencv2/opencv.hpp>
#include <QImage>
class MatConvertQImage
{

public:

    MatConvertQImage();

    ~MatConvertQImage();

    static cv::Mat QImage2cvMat(QImage image);

    static QImage Mat2QImage(const cv::Mat& InputMat);
};

#endif // MATCONVERTQIMAGE_H
