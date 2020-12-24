#define  _USE_MATH_DEFINES
#include "test.h"
#include "modelsequence.h"
#include "common/imageprocess.h"
#include "util/eigenfunctions.h"
#include <cmath>
Test::Test()
{

}

void Test::testGenerateModel()
{
    ModelSequence model;
    model.W=2048;
    model.H=2048;
//    model.Init("data\\ManHead0616.npz","data\\shape_predictor_68_face_landmarks.dat",
//               "data\\Face.png","data\\Head.png","","data\\FaceMask1.png","data\\ManHead0616.ini");
    model.Init("data\\WomenFullHead1216.npz","data\\shape_predictor_68_face_landmarks.dat",
               "data\\WomanFace.png","data\\WomanHead.jpg","","data\\WomanMask1.png","data\\women.ini");

    //model.readOnePic("F:\\Data\\50.png");
    model.readVideo("F:\\Data\\video\\VID_20200324_175555.mp4");
//    Eigen::VectorXf v=Eigen::VectorXf::Ones(4,1);
//    v(0)=4;
//    v(1)=3;
//    v(2)=2;
//    std::cout<<v<<std::endl;
//    Eigen::MatrixXf A=v.asDiagonal();
//    std::cout<<A<<std::endl;
//    std::vector<int> t1={1,2,3,4,5,6};
//     std::vector<int> t2={2,3,4,7,8};
//     std::vector<int> t=SetOperator<int>::setUnion(t1,t2);
//     std::for_each(t.begin(),t.end(),[&](int a){std::cout<<a<<std::endl;});
//    Eigen::Matrix3f R=Eigen::Matrix3f::Identity(3,3);
//    R(0,0)=std::sqrt(2)/2,R(0,1)=-std::sqrt(2)/2;
//    R(1,0)=std::sqrt(2)/2,R(1,1)=std::sqrt(2)/2;
//    std::cout<<"R:"<<R<<std::endl;
//    //std::cout<<"R.determinant():"<<R.determinant()<<std::endl;
//    AngleAxisf v(M_PI / 4, Vector3f(0, 0, 1));;
//    Eigen::Matrix3f R1=v.matrix();
//    std::cout<<"R1:"<<R1<<std::endl;
//    AngleAxisf V2;
//    V2.fromRotationMatrix(R1);
//    cout << "Rotation_vector2" << endl << V2.matrix() << endl;
//    Quaternionf Q1(cos((M_PI / 4) / 2), 0 * sin((M_PI / 4) / 2), 0 * sin((M_PI / 4) / 2), 1 * sin((M_PI / 4) / 2));
//    //第一种输出四元数的方式
//    cout << "Quaternion1" << endl << Q1.coeffs() << endl;

//    //第二种输出四元数的方式
//    cout << Q1.x() << endl << endl;
//    cout << Q1.y() << endl << endl;
//    cout << Q1.z() << endl << endl;
//    cout << Q1.w() << endl << endl;
//    //3. 使用四元数转旋转矩阵来对旋转矩阵赋值

//    //3.1 使用四元数的成员函数matrix()来对旋转矩阵赋值
//    Matrix3f R4;
//    R4 = Q1.matrix();
//    cout << "Rotation_matrix4" << endl << R4 << endl;

//    //3.2 使用四元数的成员函数toRotationMatrix()来对旋转矩阵赋值
//    Matrix3f R5;
//    R5 = Q1.toRotationMatrix();
//    cout << "Rotation_matrix5" << endl << R5 << endl;
//    Quaternionf v1(1, 0 , 0 , 0);
//    Quaternionf v2(cos((M_PI / 3) / 2), 0 * sin((M_PI / 3) / 2), 0 * sin((M_PI / 3) / 2), 1 * sin((M_PI / 3) / 2));
//    Quaternionf v3=v1.slerp(0.5,v2);
//    cout << "v3:" << endl << v3.coeffs() << endl;
//    Matrix3f R;
//    R = v3.matrix();
//    cout << "R:" << endl << R << endl;
}
