#pragma once
#define TODO
#define errorcout cout
#define debugcout cout
#define CV_AA -1
#include "cnpy.h"
#include "facemodel.h"

using namespace cv;
inline cv::Point2f ImageCoordinate(MatF face, int i)
{
    float s = 2.0;
    float x = face(3 * i + 0) * s;
    float y = face(3 * i + 1) * s;

    Point2f p(x, -y);
    Point2f offset(250, 250);

    return p + offset;
}

inline Point Shape(MatF A)
{
    return Point(A.rows(), A.cols());
}

inline Point Shape(MatI A)
{
    return Point(A.rows(), A.cols());
}

inline Eigen::Matrix3f Orthogonalize(MatF R)
{
    // Set R to the closest orthonormal matrix to the estimated affine transform:
    Eigen::JacobiSVD<Eigen::Matrix3f, Eigen::NoQRPreconditioner> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f U = svd.matrixU();
    const Eigen::Matrix3f V = svd.matrixV();
    Eigen::Matrix3f R_ortho = U * V.transpose();
    // The determinant of R must be 1 for it to be a valid rotation matrix
    if (R_ortho.determinant() < 0)
    {
        U.block<1, 3>(2, 0) = -U.block<1, 3>(2, 0); // not tested
        R_ortho = U * V.transpose();
    }
    return R_ortho;
}

inline Eigen::VectorXf SolveLinear(MatF A, MatF B, float lambda)
{
    //lambda = 1.0
    // Solve d[(Ax-b)^2 + lambda * x^2 ]/dx = 0
    // https://math.stackexchange.com/questions/725185/minimize-a-x-b
    Eigen::MatrixXf Diagonal = Eigen::MatrixXf::Identity(A.cols(), A.cols()) * lambda;
    auto AA = A.transpose() * A + Diagonal;
    Eigen::VectorXf X = AA.colPivHouseholderQr().solve(A.transpose() * B);
    return X;
}
inline Eigen::VectorXf SolveLinear(MatF A, MatF B, VectorXf rv)
{
    //lambda = 1.0
    // Solve d[(Ax-b)^2 + lambda * x^2 ]/dx = 0
    // https://math.stackexchange.com/questions/725185/minimize-a-x-b
    int n=A.cols();
    if(rv.rows()!=n){
        std::cerr<<"cols is not match!"<<std::endl;
        exit(EXIT_FAILURE);
    }
    Eigen::MatrixXf Diagonal = rv.asDiagonal();
    auto AA = A.transpose() * A + Diagonal;
    Eigen::VectorXf X = AA.colPivHouseholderQr().solve(A.transpose() * B);
    return X;
}
inline Eigen::VectorXf SolveLinear(MatF A, MatF B)
{
    Eigen::VectorXf X = A.colPivHouseholderQr().solve(B);
    return X;
}


class ProjectionParameters
{
public:
    Eigen::Matrix3f R; ///< 3x3 rotation matrix
    float tx, ty,tz; ///< x and y translation
    float s;      ///< Scaling

    //Need Transpose
    Mat GenerateCVProj()
    {
        Matrix3f Rt = R.transpose() * s;

        Mat P = Mat::eye(4, 4, CV_32F);

        for (size_t i = 0; i < 3; i++)
        {
            for (size_t j = 0; j < 3; j++)
            {
                P.at<float>(i, j) = Rt(i, j);
            }
        }

        P.at<float>(0, 3) = tx * s;
        P.at<float>(1, 3) = ty * s;

        return P;
    }
};


inline MatF Projection(ProjectionParameters p, MatF model_points)
{
    MatF R = p.R.transpose() * p.s;
    R = R.block(0, 0, 3, 2);

    MatF rotated = model_points * R;

    auto sTx = p.tx * p.s;
    auto sTy = p.ty * p.s;

    int N = model_points.rows();
    rotated.col(0).array() += sTx;
    rotated.col(1).array() += sTy;

    return rotated;// .block(0, 0, N, 2);
}

inline MatF Rotation(ProjectionParameters p, MatF model_points)
{
    MatF R = p.R;
    R = R.block(0, 0, 3, 3);
    MatF rotated = model_points * R;
    return rotated;// .block(0, 0, N, 2);
}


inline MatF ToEigen(std::vector<Point> image_points)
{
    int N = image_points.size();

    MatF b(N, 2);
    for (int i = 0; i < N; ++i)
    {
        Eigen::Vector2f p = Eigen::Vector2f(image_points[i].x, image_points[i].y);
        b.block<1, 2>(i, 0) = p;
    }

    return b;
}

inline Matrix<float, 1, Eigen::Dynamic> GetMean(MatF &input)
{
    return input.colwise().mean();
}

inline void SubtractMean(MatF &input)
{
    input.rowwise() -= GetMean(input);
}



class MMSolver
{
public:
    bool USEWEIGHT = true;
    float WEIGHT = 1.0;
    //vector<int> SkipList = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,  11, 12, 13, 14, 15, 16 };
    //vector<int> SkipList = { 0, 1, 2, 3, 4,   5, 6, 7,    9,10,  11, 12,  13, 14, 15, 16 };
    vector<int> SkipList = {8, };
    void Initialize(string file, string file2);
    float fx=1.0;
    FaceModel FM;
    FaceModel FMFull;
    bool FIXFIRSTSHAPE = false;


    MatF SX;
    MatF EX;
    ProjectionParameters params;

    bool FixShape = false;
    MatF SX0;
    MatF Transform(ProjectionParameters p, MatF model_points);
    MatF PerspectiveProjection(ProjectionParameters p, MatF model_points);
    MatF SolveShapePerspective(ProjectionParameters p, MatF image_points, MatF M, MatF SB, float lambda);
    MatF SolveShape(ProjectionParameters p, MatF image_points, MatF M, MatF SB, float lambda);
    MatF SolveShape2(ProjectionParameters p, MatF pre, MatF cur, MatF SB, float lambda);
    MatF SolveShape3(ProjectionParameters p, MatF pre, MatF cur, MatF SB, VectorXf variance);
    ProjectionParameters SolveProjectionNonlinear(MatF image_points, MatF model_points);
    ProjectionParameters SolveProjection(MatF image_points, MatF model_points);
    void Solve(MatF KP);
    void SolvePerspective(MatF KP);
    void SolvePerspective2(MatF KP);
};





inline Mat MMSDraw(Mat orig, MMSolver &MMS, MatF KP,bool center=false)
{

    auto params = MMS.params;
    auto Face2 = MMS.FMFull.Generate(MMS.SX, MMS.EX);
    //	MatF projected = Projection(params, Face2);
    MatF projected = MMS.PerspectiveProjection(params, Face2);
    if(center){
        projected.col(0).array()+=orig.cols/2;
        projected.col(1).array()+=orig.rows/2;
    }
    auto image = orig.clone();
    auto image2 = orig.clone();

    auto Ev = MMS.FMFull.Ev;

    for (size_t i = 0; i < Ev.rows(); i++)
    {
        int i1 = Ev(i, 0);
        int i2 = Ev(i, 1);

        auto x = projected(i1, 0);
        auto y = projected(i1, 1);

        auto x2 = projected(i2, 0);
        auto y2 = projected(i2, 1);

        line(image, Point(x, y), Point(x2, y2), Scalar(0, 0, 255, 255), 1);
        //image.at<Vec3b>(y, x) = Vec3b(0, 0, 255);
        //circle(image, Point(x, y), 1, Scalar(0, 0, 255), -1);
    }


    auto Face = MMS.FM.Generate(MMS.SX, MMS.EX);
    //	projected = Projection(params, Face);
    projected = MMS.PerspectiveProjection(params, Face);
    if(center){
        projected.col(0).array()+=orig.cols/2;
        projected.col(1).array()+=orig.rows/2;
    }
    for (size_t i = 0; i < projected.rows(); i++)
    {
        auto x = projected(i, 0);
        auto y = projected(i, 1);
        circle(image, Point(x, y), 8, Scalar(255, 0, 0, 255), -1, CV_AA);
    }

    if (MMS.USEWEIGHT)
    {
        for (size_t i = 0; i < MMS.SkipList.size(); i++)
        {
            int i2 = MMS.SkipList[i];
            auto x = projected(i2, 0);
            auto y = projected(i2, 1);
            circle(image, Point(x, y), 24, Scalar(255, 0, 0, 255), 1, CV_AA);
        }
    }
    if(center){
        KP.col(0).array()+=orig.cols/2;
        KP.col(1).array()+=orig.rows/2;
    }
    for (size_t i = 0; i < KP.rows(); i++)
    {
        auto x = KP(i, 0);
        auto y = KP(i, 1);

        circle(image, Point(x, y), 8, Scalar(0, 255, 0, 255), -1, CV_AA);
    }

    //imshow("IMG", image / 2 + image2 / 2);
    return image / 2 + image2 / 2;
}





// Apply affine transform calculated using srcTri and dstTri to src
inline void applyAffineTransform(Mat &warpImage, Mat &src, vector<Point2f> &srcTri, vector<Point2f> &dstTri)
{
    // Given a pair of triangles, find the affine transform.
    Mat warpMat = getAffineTransform(srcTri, dstTri);

    // Apply the Affine Transform just found to the src image
    warpAffine(src, warpImage, warpMat, warpImage.size(), INTER_LINEAR, BORDER_REFLECT_101);
}


// Warps and alpha blends triangular regions from img1 and img2 to img
inline void warpTriangle(Mat &img1, Mat &img2, vector<Point2f> &t1, vector<Point2f> &t2)
{
    TODO // Need to make sure rect is in Mat.
            Rect r1 = boundingRect(t1);
    Rect r2 = boundingRect(t2);
    try
    {



        // Offset points by left top corner of the respective rectangles
        vector<Point2f> t1Rect, t2Rect;
        vector<Point> t2RectInt;
        for (int i = 0; i < 3; i++)
        {

            t1Rect.push_back(Point2f(t1[i].x - r1.x, t1[i].y - r1.y));
            t2Rect.push_back(Point2f(t2[i].x - r2.x, t2[i].y - r2.y));
            t2RectInt.push_back(Point(t2[i].x - r2.x, t2[i].y - r2.y)); // for fillConvexPoly

        }

        // Get mask by filling triangle
        Mat mask = Mat::zeros(r2.height, r2.width, CV_8UC3);
        fillConvexPoly(mask, t2RectInt, Scalar(1.0, 1.0, 1.0), 8, 0);
        // Apply warpImage to small rectangular patches
        Mat img1Rect;
        img1(r1).copyTo(img1Rect);
        Mat img2Rect = Mat::zeros(r2.height, r2.width, img1Rect.type());
        applyAffineTransform(img2Rect, img1Rect, t1Rect, t2Rect);
        multiply(img2Rect, mask, img2Rect);
        multiply(img2(r2), Scalar(1.0, 1.0, 1.0) - mask, img2(r2));
        img2(r2) = img2(r2) + img2Rect;
    }
    catch (const std::exception& e) {
        std::cout<<"---------------------------"<<std::endl;
        cout << r1 << r2 << endl;
        cout<<t2[0]<<endl;
        cout<<t2[1]<<endl;
        cout<<t2[2]<<endl;
        std::cout << e.what();
        //fillConvexPoly(img2, t2, Scalar(0, 0, 255), 16, 0);
    }



}

inline Mat MMSPerspectiveTexture(Mat orig, MMSolver &MMS, int W, int H,bool center=false, bool doubleface = true)
{
    auto image = orig.clone();
    Mat texture = Mat::zeros(H, W, CV_8UC3);

    auto params = MMS.params;
    auto Face2 = MMS.FMFull.Generate(MMS.SX, MMS.EX);
    MatF projected = MMS.PerspectiveProjection(params, Face2);
    if(center){
        projected.col(0).array()+=orig.cols/2;
        projected.col(1).array()+=orig.rows/2;
    }

    auto TRI = MMS.FMFull.TRIUV_FACE;
    auto UV = MMS.FMFull.UV;

    for (size_t t = 0; t < TRI.rows(); t++)
    {

        vector<Point2f> t1;
        vector<Point2f> t2;
        bool skip=false;
        for (size_t i = 0; i < 3; i++)
        {
            int j = TRI(t, i);
            auto x = projected(j, 0);
            auto y = projected(j, 1);
            if(x<0||x>=orig.cols-1||y<0||y>=orig.rows-1){
                //std::cout<<"x:"<<x<<" y:"<<y<<std::endl;
                skip=true;
                continue;
            }
            if(UV(j, 0)<0||UV(j, 0)>1||UV(j, 1)<0||UV(j, 1)>1){
                //std::cout<<"u:"<<UV(j, 0)<<" v:"<<UV(j, 1)<<std::endl;
                skip=true;
                continue;
            }
            auto u = (UV(j, 0)) * (W - 1);
            auto v = (1 - UV(j, 1)) * (H - 1);
            t1.push_back(Point2f(x, y));
            t2.push_back(Point2f(u, v));

            //cout << Point2f(x, y) << Point2f(u, v) << endl;
        }
        if(skip)continue;
        auto c = (t1[2] - t1[0]).cross(t1[1] - t1[0]);

        if (doubleface || c > 0)
        {
            warpTriangle(image, texture, t1, t2);
        }
    }

    return texture;

}
inline Mat MMSTexture(Mat orig, MMSolver &MMS, int W, int H,bool doubleface = true)
{
    auto image = orig.clone();
    Mat texture = Mat::zeros(H, W, CV_8UC3);

    auto params = MMS.params;
    auto Face2 = MMS.FMFull.Generate(MMS.SX, MMS.EX);
    MatF projected = Projection(params, Face2);


    auto TRI = MMS.FMFull.TRIUV;
    auto UV = MMS.FMFull.UV;

    for (size_t t = 0; t < TRI.rows(); t++)
    {

        vector<Point2f> t1;
        vector<Point2f> t2;
        for (size_t i = 0; i < 3; i++)
        {
            int j = TRI(t, i);
            auto x = projected(j, 0);
            auto y = projected(j, 1);
            auto u = (UV(j, 0)) * (W - 1);
            auto v = (1 - UV(j, 1)) * (H - 1);
            t1.push_back(Point2f(x, y));
            t2.push_back(Point2f(u, v));

            //cout << Point2f(x, y) << Point2f(u, v) << endl;
        }

        auto c = (t1[2] - t1[0]).cross(t1[1] - t1[0]);

        if (doubleface || c > 0)
        {
            warpTriangle(image, texture, t1, t2);
        }
    }

    return texture;

}


inline Mat MMSNormal(Mat orig, MMSolver &MMS, int W, int H)
{
    auto image = orig.clone();
    Mat texture = Mat::zeros(H, W, CV_8UC3);

    auto params = MMS.params;
    auto Face2 = MMS.FMFull.Generate(MMS.SX, MMS.EX);
    Face2 = Rotation(params, Face2);
    auto TRI = MMS.FMFull.TRIUV;
    auto UV = MMS.FMFull.UV;

    for (size_t t = 0; t < TRI.rows(); t++)
    {

        vector<Point3f> t1;
        vector<Point> t2;
        for (size_t i = 0; i < 3; i++)
        {
            int j = TRI(t, i);
            auto x = Face2(j, 0);
            auto y = Face2(j, 1);
            auto z = Face2(j, 2);

            auto u = (UV(j, 0)) * (W - 1);
            auto v = (1.0 - UV(j, 1)) * (H - 1);
            t1.push_back(Point3f(x, y, z));
            t2.push_back(Point(u, v));

            //cout << Point2f(x, y) << Point2f(u, v) << endl;
        }

        Point3f c = (t1[2] - t1[0]).cross(t1[1] - t1[0]);
        auto normal = c / norm(c);

        /*if (c.z > 0)
        {
            float n = ((c.z / norm(c))) * 255;

            fillConvexPoly(texture, t2, Scalar(n,n,n), 8, 0);
        }*/

        {
            fillConvexPoly(texture, t2, Scalar(normal.x + 1.0, normal.y + 1.0, normal.z + 1.0) * 128, 8, 0);
        }

    }

    return texture;

}



#include <sstream>
#include <fstream>  
#include "cnpy.h"


inline void FMObj(Mat texture, FaceModel &FM, string folder, string filename0)
{
    string filename = folder + filename0;
    imwrite(filename + ".png", texture);

    //cout << Shape(MMS.SX) << Shape(MMS.EX) << endl;
    auto Face = FM.GeneratedFace;
    auto TRI = FM.TRI;
    auto TRIUV = FM.TRIUV;
    auto UV = FM.UV;

    //string numpyfile = filename + ".npz";




    {
        std::stringstream ss;


        ss << "mtllib " << filename0 << ".mtl" << endl;
        ss << "o FaceObject" << endl;

        int N = Face.rows();
        for (size_t i = 0; i < N; i++)
        {
            ss << "v " << Face(i, 0) << " " << Face(i, 1) << " " << Face(i, 2) << endl;
        }

        N = UV.rows();
        for (size_t i = 0; i < N; i++)
        {
            ss << "vt " << UV(i, 0) << " " << UV(i, 1) << endl;
        }

        ss << "usemtl material_0" << endl;
        ss << "s 1" << endl;

        N = TRI.rows();
        for (size_t i = 0; i < N; i++)
        {
            ss << "f " << TRI(i, 0) + 1 << "/" << TRIUV(i, 0) + 1 << " "
               << TRI(i, 1) + 1 << "/" << TRIUV(i, 1) + 1 << " "
               << TRI(i, 2) + 1 << "/" << TRIUV(i, 2) + 1 << " "
               << endl;
        }


        std::string input = ss.str();

        std::ofstream out(filename + ".obj", std::ofstream::out);
        out << input;
        out.close();
    }

    {
        std::stringstream ss;


        ss << "mtllib " << filename0 << ".mtl" << endl;
        ss << "o FaceObject" << endl;

        ss << "newmtl material_0" << endl;
        ss << "	Ns 0.000000" << endl;
        ss << "Ka 0.200000 0.200000 0.200000" << endl;
        ss << "Kd 0.639216 0.639216 0.639216" << endl;
        ss << "Ks 1.000000 1.000000 1.000000" << endl;
        ss << "Ke 0.000000 0.000000 0.000000" << endl;
        ss << "Ni 1.000000" << endl;
        ss << "d 1.000000" << endl;
        ss << "illum 2" << endl;
        ss << "map_Kd " << filename0 + ".png" << endl;



        std::string input = ss.str();

        std::ofstream out(filename + ".mtl", std::ofstream::out);
        out << input;
        out.close();
    }


}
inline void MMSObjWithTexture(Mat texture, MMSolver &MMS, string folder, string filename0)
{
    string filename = folder + filename0;
    auto Face = MMS.FMFull.Generate(MMS.SX, MMS.EX);
    MatI TRIUV_FACE=MMS.FMFull.TRIUV_FACE;
    MatI TRIUV_HEAD=MMS.FMFull.TRIUV_HEAD;
//    MatI TRI_BODY=MMS.FMFull.TRI_BODY;
//    MatI TRIUV_BODY=MMS.FMFull.TRIUV_BODY;

    int headOffset=MMS.FMFull.HEAD_OFFSET(0,0);
//    int bodyOffset=MMS.FMFull.BODY_OFFSET(0,0);
    auto UV = MMS.FMFull.UV;
    string numpyfile = filename + ".npz";

    std::string pic0=filename0+"Face";
    std::string pic1=filename0+"Head";
//    std::string pic2=filename0+"Body";
    //cnpy::npz_save(numpyfile, "SX", MMS.SX.data(), { (unsigned long long)MMS.SX.rows(), (unsigned long long)MMS.SX.cols() }, "w"); //"w" overwrites any existing file
    //cnpy::npz_save(numpyfile, "EX", MMS.EX.data(), { (unsigned long long)MMS.EX.rows(), (unsigned long long)MMS.EX.cols() }, "a"); //"a" appends to the file we created above

    {
        std::stringstream ss;


        ss << "mtllib " << filename0 << ".mtl" << endl;

        ss << "o FaceObject" << endl;
        int N = Face.rows();
        for (size_t i = 0; i < /*N*/headOffset; i++)
        {
            ss << "v " << Face(i, 0) << " " << Face(i, 1) << " " << Face(i, 2) << endl;
            ss << "vt " << UV(i, 0) << " " << UV(i, 1) << endl;
        }

        ss << "usemtl "<<pic0 << endl;
        ss << "s 1" << endl;

        N = TRIUV_FACE.rows();
        for (size_t i = 0; i < N; i++)
        {
            ss << "f " << TRIUV_FACE(i, 0) + 1 << "/" << TRIUV_FACE(i, 0) + 1 << " "
               << TRIUV_FACE(i, 1) + 1 << "/" << TRIUV_FACE(i, 1) + 1 << " "
               << TRIUV_FACE(i, 2) + 1 << "/" << TRIUV_FACE(i, 2) + 1 << " "
               << endl;
        }

        ss << "o HeadObject" << endl;
        N=Face.rows();
        for (size_t i = headOffset; i < N/*bodyOffset*/; i++)
        {
            ss << "v " << Face(i, 0) << " " << Face(i, 1) << " " << Face(i, 2) << endl;
            ss << "vt " << UV(i, 0) << " " << UV(i, 1) << endl;
        }
        ss << "usemtl "<<pic1 << endl;
        ss << "s 1" << endl;
        N = TRIUV_HEAD.rows();
        for (size_t i = 0; i < N; i++)
        {
            ss << "f " << TRIUV_HEAD(i, 0) + 1 << "/" << TRIUV_HEAD(i, 0) + 1 << " "
               << TRIUV_HEAD(i, 1) + 1 << "/" << TRIUV_HEAD(i, 1) + 1 << " "
               << TRIUV_HEAD(i, 2) + 1 << "/" << TRIUV_HEAD(i, 2) + 1 << " "
               << endl;
        }

//        ss << "o BodyObject" << endl;
//        N = Face.rows();
//        for (size_t i = bodyOffset; i < N; i++)
//        {
//            ss << "v " << Face(i, 0) << " " << Face(i, 1) << " " << Face(i, 2) << endl;
//        }
//        N = UV.rows();
//        for (size_t i = bodyOffset; i < N; i++)
//        {
//            ss << "vt " << UV(i, 0)/4 << " " << UV(i, 1)/4 << endl;
//        }

//        ss << "usemtl "<<pic2 << endl;
//        ss << "s 1" << endl;
//        N = TRI_BODY.rows();
//        for (size_t i = 0; i < N; i++)
//        {
//            ss << "f " << TRI_BODY(i, 0) + 1 << "/" << TRIUV_BODY(i, 0) + 1 << " "
//               << TRI_BODY(i, 1) + 1 << "/" << TRIUV_BODY(i, 1) + 1 << " "
//               << TRI_BODY(i, 2) + 1 << "/" << TRIUV_BODY(i, 2) + 1 << " "
//               << endl;
//        }

        std::string input = ss.str();

        std::ofstream out(filename + ".obj", std::ofstream::out);
        out << input;
        out.close();
    }

    {
        std::stringstream ss;
        ss << "mtllib " << filename0 << ".mtl" << endl;

        ss << "o FaceObject" << endl;
        ss << "newmtl "<<pic0 << endl;
        ss << "	Ns 0.000000" << endl;
        ss << "Ka 0.200000 0.200000 0.200000" << endl;
        ss << "Kd 0.639216 0.639216 0.639216" << endl;
        ss << "Ks 1.000000 1.000000 1.000000" << endl;
        ss << "Ke 0.000000 0.000000 0.000000" << endl;
        ss << "Ni 1.000000" << endl;
        ss << "d 1.000000" << endl;
        ss << "illum 2" << endl;
        ss << "map_Kd " << pic0 + ".png" << endl;

        ss << "o HeadObject" << endl;
        ss << "newmtl "<<pic1 << endl;
        ss << "	Ns 0.000000" << endl;
        ss << "Ka 0.200000 0.200000 0.200000" << endl;
        ss << "Kd 0.639216 0.639216 0.639216" << endl;
        ss << "Ks 1.000000 1.000000 1.000000" << endl;
        ss << "Ke 0.000000 0.000000 0.000000" << endl;
        ss << "Ni 1.000000" << endl;
        ss << "d 1.000000" << endl;
        ss << "illum 2" << endl;
        ss << "map_Kd " << pic1 + ".png" << endl;

//        ss << "o BodyObject" << endl;
//        ss << "newmtl "<<pic2 << endl;
//        ss << "	Ns 0.000000" << endl;
//        ss << "Ka 0.200000 0.200000 0.200000" << endl;
//        ss << "Kd 0.639216 0.639216 0.639216" << endl;
//        ss << "Ks 1.000000 1.000000 1.000000" << endl;
//        ss << "Ke 0.000000 0.000000 0.000000" << endl;
//        ss << "Ni 1.000000" << endl;
//        ss << "d 1.000000" << endl;
//        ss << "illum 2" << endl;
//        ss << "map_Kd " << pic2 + ".png" << endl;



        std::string input = ss.str();

        std::ofstream out(filename + ".mtl", std::ofstream::out);
        out << input;
        out.close();
    }
}

inline void MMSObj(Mat orig, MMSolver &MMS, string folder, string filename0)
{
    string filename = folder + filename0;

    Mat texture = MMSTexture(orig, MMS, 1024,1024);
    imwrite(filename + ".png", texture);

    auto Face = MMS.FMFull.Generate(MMS.SX, MMS.EX);
    auto TRI = MMS.FMFull.TRI;
    auto TRIUV = MMS.FMFull.TRIUV;
    auto UV = MMS.FMFull.UV;

    string numpyfile = filename + ".npz";


    cnpy::npz_save(numpyfile, "SX", MMS.SX.data(), { (unsigned long long)MMS.SX.rows(), (unsigned long long)MMS.SX.cols() }, "w"); //"w" overwrites any existing file
    cnpy::npz_save(numpyfile, "EX", MMS.EX.data(), { (unsigned long long)MMS.EX.rows(), (unsigned long long)MMS.EX.cols() }, "a"); //"a" appends to the file we created above

    {
        std::stringstream ss;


        ss << "mtllib " << filename0 << ".mtl" << endl;
        ss << "o FaceObject" << endl;

        int N = Face.rows();
        for (size_t i = 0; i < N; i++)
        {
            ss << "v " << Face(i, 0) << " " << Face(i, 1) << " " << Face(i, 2) << endl;
            ss << "vt " << UV(i, 0) << " " << UV(i, 1) << endl;
        }

        ss << "usemtl material_0" << endl;
        ss << "s 1" << endl;

        N = TRI.rows();
        for (size_t i = 0; i < N; i++)
        {
            ss << "f " << TRI(i, 0) + 1 << "/" << TRIUV(i, 0) + 1 << " "
               << TRI(i, 1) + 1 << "/" << TRIUV(i, 1) + 1 << " "
               << TRI(i, 2) + 1 << "/" << TRIUV(i, 2) + 1 << " "
               << endl;
        }


        std::string input = ss.str();

        std::ofstream out(filename + ".obj", std::ofstream::out);
        out << input;
        out.close();
    }

    {
        std::stringstream ss;


        ss << "mtllib " << filename0 << ".mtl" << endl;
        ss << "o FaceObject" << endl;

        ss << "newmtl material_0" << endl;
        ss << "	Ns 0.000000" << endl;
        ss << "Ka 0.200000 0.200000 0.200000" << endl;
        ss << "Kd 0.639216 0.639216 0.639216" << endl;
        ss << "Ks 1.000000 1.000000 1.000000" << endl;
        ss << "Ke 0.000000 0.000000 0.000000" << endl;
        ss << "Ni 1.000000" << endl;
        ss << "d 1.000000" << endl;
        ss << "illum 2" << endl;
        ss << "map_Kd " << filename0 + ".png" << endl;



        std::string input = ss.str();

        std::ofstream out(filename + ".mtl", std::ofstream::out);
        out << input;
        out.close();
    }


}
inline void MMSObj2(MatF Face, MMSolver &MMS, string folder, string filename0)
{
    string filename = folder + filename0;
    MatI TRIUV_FACE=MMS.FMFull.TRIUV_FACE;
    MatI TRIUV_HEAD=MMS.FMFull.TRIUV_HEAD;


    int headOffset=MMS.FMFull.HEAD_OFFSET(0,0);
    auto UV = MMS.FMFull.UV;
    string numpyfile = filename + ".npz";

    std::string pic0=filename0+"Face";
    std::string pic1=filename0+"Head";

    {
        std::stringstream ss;


        ss << "mtllib " << filename0 << ".mtl" << endl;

        ss << "o FaceObject" << endl;
        int N = Face.rows();
        for (size_t i = 0; i < /*N*/headOffset; i++)
        {
            ss << "v " << Face(i, 0) << " " << Face(i, 1) << " " << Face(i, 2) << endl;
            ss << "vt " << UV(i, 0) << " " << UV(i, 1) << endl;
        }

        ss << "usemtl "<<pic0 << endl;
        ss << "s 1" << endl;

        N = TRIUV_FACE.rows();
        for (size_t i = 0; i < N; i++)
        {
            ss << "f " << TRIUV_FACE(i, 0) + 1 << "/" << TRIUV_FACE(i, 0) + 1 << " "
               << TRIUV_FACE(i, 1) + 1 << "/" << TRIUV_FACE(i, 1) + 1 << " "
               << TRIUV_FACE(i, 2) + 1 << "/" << TRIUV_FACE(i, 2) + 1 << " "
               << endl;
        }

        ss << "o HeadObject" << endl;
        N=Face.rows();
        for (size_t i = headOffset; i < N/*bodyOffset*/; i++)
        {
            ss << "v " << Face(i, 0) << " " << Face(i, 1) << " " << Face(i, 2) << endl;
            ss << "vt " << UV(i, 0) << " " << UV(i, 1) << endl;
        }
        ss << "usemtl "<<pic1 << endl;
        ss << "s 1" << endl;
        N = TRIUV_HEAD.rows();
        for (size_t i = 0; i < N; i++)
        {
            ss << "f " << TRIUV_HEAD(i, 0) + 1 << "/" << TRIUV_HEAD(i, 0) + 1 << " "
               << TRIUV_HEAD(i, 1) + 1 << "/" << TRIUV_HEAD(i, 1) + 1 << " "
               << TRIUV_HEAD(i, 2) + 1 << "/" << TRIUV_HEAD(i, 2) + 1 << " "
               << endl;
        }



        std::string input = ss.str();

        std::ofstream out(filename + ".obj", std::ofstream::out);
        out << input;
        out.close();
    }

    {
        std::stringstream ss;
        ss << "mtllib " << filename0 << ".mtl" << endl;

        ss << "o FaceObject" << endl;
        ss << "newmtl "<<pic0 << endl;
        ss << "	Ns 0.000000" << endl;
        ss << "Ka 0.200000 0.200000 0.200000" << endl;
        ss << "Kd 0.639216 0.639216 0.639216" << endl;
        ss << "Ks 1.000000 1.000000 1.000000" << endl;
        ss << "Ke 0.000000 0.000000 0.000000" << endl;
        ss << "Ni 1.000000" << endl;
        ss << "d 1.000000" << endl;
        ss << "illum 2" << endl;
        ss << "map_Kd " << pic0 + ".png" << endl;

        ss << "o HeadObject" << endl;
        ss << "newmtl "<<pic1 << endl;
        ss << "	Ns 0.000000" << endl;
        ss << "Ka 0.200000 0.200000 0.200000" << endl;
        ss << "Kd 0.639216 0.639216 0.639216" << endl;
        ss << "Ks 1.000000 1.000000 1.000000" << endl;
        ss << "Ke 0.000000 0.000000 0.000000" << endl;
        ss << "Ni 1.000000" << endl;
        ss << "d 1.000000" << endl;
        ss << "illum 2" << endl;
        ss << "map_Kd " << pic1 + ".png" << endl;




        std::string input = ss.str();

        std::ofstream out(filename + ".mtl", std::ofstream::out);
        out << input;
        out.close();
    }
}

