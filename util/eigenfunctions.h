#ifndef EIGENFUNCTIONS_H
#define EIGENFUNCTIONS_H
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>
#include <fstream>
#include <cmath>
template <typename Type,int _Options=Eigen::ColMajor>
class EigenFunctions
{
public:
    typedef typename  Eigen::Matrix<Type,3, 1> Vector3T;
    static void xRotation180(Eigen::Matrix<Type, 3, Eigen::Dynamic>& points);
    static void yRotation180(Eigen::Matrix<Type, 3, Eigen::Dynamic>& points);
    static void zRotation180(Eigen::Matrix<Type, 3, Eigen::Dynamic>& points);
    static Eigen::Matrix<Type, 3, 3> computeCovariance(Eigen::Matrix<Type, 3, Eigen::Dynamic> points,Eigen::Matrix<Type, 3, 1> center);
    static void svdDecomposition(Eigen::Matrix<Type, 3,3> &A,Eigen::Matrix<Type, 3, 3> &U,Eigen::Matrix<Type, 3, 3> &S,Eigen::Matrix<Type, 3, 3> &V);
    static void saveEigenPoints(Eigen::Matrix<Type,3,Eigen::Dynamic>points,Eigen::Matrix3Xi faces,std::string savePath);
    static void saveEigenPoints(Eigen::Matrix<Type,3,Eigen::Dynamic>points,Eigen::Matrix3Xi faces,Eigen::Matrix4Xi colors,std::string savePath);
    static void saveEigenPoints(Eigen::Matrix<Type,3,Eigen::Dynamic>points,Eigen::Matrix3Xi faces,Eigen::Matrix3Xf colors,std::string savePath);
    static void saveEigenPointsWithUV(Eigen::Matrix<Type,3,Eigen::Dynamic>points,Eigen::Matrix3Xi faces,Eigen::Matrix2Xf uvs,std::string savePath);
    static void saveEigenPoints(Eigen::Matrix<Type,3,Eigen::Dynamic>points,Eigen::Matrix4Xi polygons,Eigen::Matrix4Xi colors,std::string savePath);
    static void saveEigenPoints(std::vector<Type>&points,std::vector<int> faces,std::string savePath);
    static void saveEigenPointsWithColors(std::vector<Type>&points,std::vector<int> faces,std::vector<int> colors,std::string savePath);
    static void saveEigenPointsWithKey(std::vector<Type>&points,std::vector<int> faces,std::vector<int> keys,std::string savePath);
    static void saveEigenPointsWithUV(std::vector<Type>&points,std::vector<int> faces,std::vector<Type> uvs,std::string savePath);
    static void saveEigenPointsWithKey(Eigen::Matrix<Type,3,Eigen::Dynamic>&points,Eigen::Matrix3Xi faces,std::vector<int> keys,std::string savePath);
    static void saveEigenPointsWithKey(Eigen::Matrix<Type,3,Eigen::Dynamic>&points,Eigen::Matrix4Xi polygons,std::vector<int> keys,std::string savePath);
    static void saveEigenPointsWithNormals(Eigen::Matrix<Type,3,Eigen::Dynamic>points,Eigen::Matrix3Xi faces,Eigen::Matrix <Type, 3, Eigen::Dynamic> normals,std::string savePath);

    static void saveEigenPointsWithWeights(Eigen::Matrix<Type,3,Eigen::Dynamic>&points,Eigen::Matrix3Xi faces,std::vector<float> weights,std::string savePath);
    static Type computeScale(Eigen::Matrix<Type,3,Eigen::Dynamic> points);
    static void center(Eigen::Matrix<Type,3,Eigen::Dynamic>& points);
    static void polygons2faces(const Eigen::Matrix<Type,4,Eigen::Dynamic>& polygons,Eigen::Matrix<Type,3,Eigen::Dynamic>& faces);
    static Eigen::Matrix3Xf computeFaceNormals( Eigen::Matrix3Xf points, Eigen::Matrix3Xi faces);
    static Eigen::SparseMatrix<float> computeFaceNormalsWeight(int64_t vnum, Eigen::Matrix3Xi faces);
    static Eigen::Matrix3Xf computeVertNormals( Eigen::Matrix3Xf points,Eigen::Matrix3Xi faces);
    static Eigen::Matrix<Type,Eigen::Dynamic,Eigen::Dynamic,_Options> vector2Matrix(const std::vector<Type>& vec,int rows,int cols);
    static std::tuple<Eigen::Matrix<Type,3,Eigen::Dynamic>,Eigen::Matrix3Xi> mergeObjs(std::vector<Eigen::Matrix<Type,3,Eigen::Dynamic>>&objs,std::vector<Eigen::Matrix3Xi>&faces);
    static Eigen::Matrix<Type,3,3> EigenFunctions<Type,_Options>::rodrigues(Vector3T &theta);
    static Vector3T EigenFunctions::unRodrigues(Eigen::Matrix<Type,3,3> &R);
    //static Eigen::Quaternion<Type> Slerp(Eigen::Quaternion<Type>& v0,Eigen::Quaternion<Type>& v1,Type t=0.5);
};
/**
 * @brief EigenFunctions::rodrigues
 * @param theta 3x1
 * @return      3x3
 */
template<typename Type, int _Options>
Eigen::Matrix<Type,3,3> EigenFunctions<Type,_Options>::rodrigues(Vector3T &theta)
{
    assert(theta.rows() == 3);
    Type angle=theta.norm();//1
    Eigen::Matrix<Type,3,Eigen::Dynamic> I=Eigen::Matrix<Type,3,Eigen::Dynamic>::Identity(3,3);
    if(angle<1e-6){
        return I;
    }

    Vector3T normalized=theta/angle;//3x1
    Type nxs=normalized[0];//1
    Type nys=normalized[1];//1
    Type nzs=normalized[2];//1
    Eigen::Matrix<Type,3,Eigen::Dynamic> k=Eigen::Matrix<Type,3,Eigen::Dynamic>::Zero(3,3);
    k<<0,-nzs,nys,
            nzs,0,-nxs,
            -nys,nxs,0;
    Eigen::Matrix<Type,3,Eigen::Dynamic> dot=normalized*normalized.transpose();
    Type cos=std::cos(angle);//1
    Type sin=std::sin(angle);//1
    Eigen::Matrix<Type,3,Eigen::Dynamic> R=cos*I+(1-cos)*dot+sin*k;
    return std::move(R);
}
/**
 * @brief EigenFunctions::unRodrigues
 * @param R 3x3
 * @return  3x1
 */
template<typename Type, int _Options>
typename EigenFunctions<Type,_Options>::Vector3T EigenFunctions<Type,_Options>::unRodrigues(Eigen::Matrix<Type,3,3> &R)
{
    Eigen::Matrix<Type,3,3> temp=(R-R.transpose())/2;
    Vector3T v(temp(2,1),temp(0,2),temp(1,0));
    Type sin=v.norm();
    Type theta=std::asin(sin);
    if(theta<1e-6){
        return std::move(v);
    }else{
        return std::move(v/sin*theta);
    }
}
template<typename Type, int _Options>
void EigenFunctions<Type,_Options>::saveEigenPointsWithNormals(Eigen::Matrix<Type, 3, Eigen::Dynamic> points, Eigen::Matrix3Xi faces,Eigen::Matrix <Type, 3, Eigen::Dynamic> normals, std::string savePath)
{
    std::ofstream out(savePath);
    if(!out){
        std::cerr<<"write file:"<<savePath<<" fail!"<<std::endl<<std::flush;
        return;
    }
    if(points.size()==0){
        std::cerr<<"points is empty!"<<std::endl<<std::flush;
        return;
    }

    for (int i = 0; i < points.cols(); i++) {
        out << "v " << points(0,i)<<" " << points(1,i) << " " << points(2,i);
        out<< std::endl;
    }
    for (int i = 0; i < normals.cols(); i++) {
        out << "vn " << normals(0,i)<<" " << normals(1,i) << " " << normals(2,i);
        out<< std::endl;
    }
    //std::cout<<"------------"<<std::endl<<std::flush;
    for(int i=0;i<faces.cols();i++){
        out << "f " << faces(0,i)+1<<" " << faces(1,i)+1 << " " << faces(2,i)+1 << std::endl;;
    }
    out.close();
}
template<typename Type,int _Options>
void EigenFunctions<Type,_Options>::xRotation180(Eigen::Matrix<Type, 3, Eigen::Dynamic> &points)
{
    Eigen::Matrix<Type, 3, 3> R=-Eigen::Matrix<float, 3, 3>::Identity();
    R(0,0)=1;
    points=R*points;
}
template<typename Type,int _Options>
void EigenFunctions<Type,_Options>::yRotation180(Eigen::Matrix<Type, 3, Eigen::Dynamic> &points)
{
    Eigen::Matrix<Type, 3, 3> R=-Eigen::Matrix<float, 3, 3>::Identity();
    R(1,1)=1;
    points=R*points;
}
template<typename Type,int _Options>
void EigenFunctions<Type,_Options>::zRotation180(Eigen::Matrix<Type, 3, Eigen::Dynamic> &points)
{
    Eigen::Matrix<Type, 3, 3> R=-Eigen::Matrix<float, 3, 3>::Identity();
    R(2,2)=1;
    points=R*points;
}
template<typename Type,int _Options>
Eigen::Matrix<Type, 3, 3> EigenFunctions<Type,_Options>::computeCovariance(Eigen::Matrix<Type, 3, Eigen::Dynamic> points, Eigen::Matrix<Type, 3, 1> center)
{
    points.colwise()-=center;
    return std::move(points*points.transpose());
}

template<typename Type,int _Options>
void EigenFunctions<Type,_Options>::svdDecomposition(Eigen::Matrix<Type, 3, 3> &A, Eigen::Matrix<Type, 3, 3> &U,Eigen::Matrix<Type, 3, 3> &S, Eigen::Matrix<Type, 3, 3> &V)
{
    using namespace Eigen;
    using namespace Eigen::internal;
    using namespace Eigen::Architecture;
    JacobiSVD<Eigen::Matrix<Type, Dynamic, Dynamic> > svd(A, ComputeThinU | ComputeThinV );
    V = svd.matrixV(), U = svd.matrixU();
    S = U.transpose() * A * V; // S = U^-1 * A * VT * -1
}

template<typename Type, int _Options>
void EigenFunctions<Type,_Options>::saveEigenPoints(Eigen::Matrix<Type, 3, Eigen::Dynamic> points, Eigen::Matrix3Xi faces, std::string savePath)
{
    std::ofstream out(savePath/*, std::ios::trunc*/);
    if(!out){
        std::cerr<<"write file:"<<savePath<<" fail!"<<std::endl<<std::flush;
        return;
    }
    if(points.size()==0){
        std::cerr<<"points is empty!"<<std::endl<<std::flush;
        return;
    }

    for (int i = 0; i < points.cols(); i++) {
        out << "v " << points(0,i)<<" " << points(1,i) << " " << points(2,i) << std::endl;
    }
    for(int i=0;i<faces.cols();i++){
        out << "f " << faces(0,i)+1<<" " << faces(1,i)+1 << " " << faces(2,i)+1 << std::endl;;
    }
    out.close();
}

template<typename Type,int _Options>
void EigenFunctions<Type,_Options>::saveEigenPoints(Eigen::Matrix<Type, 3, Eigen::Dynamic> points,Eigen::Matrix3Xi faces,Eigen::Matrix4Xi colors, std::string savePath)
{
    std::ofstream out(savePath/*, std::ios::trunc*/);
    if(!out){
        std::cerr<<"write file:"<<savePath<<" fail!"<<std::endl<<std::flush;
        return;
    }
    if(points.size()==0){
        std::cerr<<"points is empty!"<<std::endl<<std::flush;
        return;
    }

    bool hasColor=colors.size()>0;
    for (int i = 0; i < points.cols(); i++) {
        out << "v " << points(0,i)<<" " << points(1,i) << " " << points(2,i) << " ";
        if(hasColor){
            out << colors(0,i)<<" " << colors(1,i) << " " << colors(2,i) << " "<< colors(3,i);
        }
        out<< std::endl;
    }
    for(int i=0;i<faces.cols();i++){
        out << "f " << faces(0,i)+1<<" " << faces(1,i)+1 << " " << faces(2,i)+1 << std::endl;;
    }
    out.close();
}

template<typename Type, int _Options>
void EigenFunctions<Type,_Options>::saveEigenPoints(Eigen::Matrix<Type, 3, Eigen::Dynamic> points, Eigen::Matrix3Xi faces, Eigen::Matrix3Xf colors, std::string savePath)
{
    std::ofstream out(savePath/*, std::ios::trunc*/);
    if(!out){
        std::cerr<<"write file:"<<savePath<<" fail!"<<std::endl<<std::flush;
        return;
    }
    if(points.size()==0){
        std::cerr<<"points is empty!"<<std::endl<<std::flush;
        return;
    }

    bool hasColor=colors.size()>0;
    for (int i = 0; i < points.cols(); i++) {
        out << "v " << points(0,i)<<" " << points(1,i) << " " << points(2,i) << " ";
        if(hasColor){
            out << colors(0,i)<<" " << colors(1,i) << " " << colors(2,i) /*<< " "<< colors(3,i)*/;
        }
        out<< std::endl;
    }
    //std::cout<<"------------"<<std::endl<<std::flush;
    for(int i=0;i<faces.cols();i++){
        out << "f " << faces(0,i)+1<<" " << faces(1,i)+1 << " " << faces(2,i)+1 << std::endl;;
    }
    out.close();
}

template<typename Type, int _Options>
void EigenFunctions<Type,_Options>::saveEigenPointsWithUV(Eigen::Matrix<Type, 3, Eigen::Dynamic> points, Eigen::Matrix3Xi faces, Eigen::Matrix2Xf uvs, std::string savePath)
{
    std::ofstream out(savePath/*, std::ios::trunc*/);
    if(!out){
        std::cerr<<"write file:"<<savePath<<" fail!"<<std::endl<<std::flush;
        return;
    }
    if(points.size()==0){
        std::cerr<<"points is empty!"<<std::endl<<std::flush;
        return;
    }
    for (int i = 0; i < points.cols(); i++) {
        out << "v " << points(0,i)<<" " << points(1,i) << " " << points(2,i) << std::endl;
        out  << "vt " << uvs(0,i)<<" " << uvs(1,i) << std::endl;

    }
    for(int i=0;i<faces.cols();i++){
        out << "f " << faces(0,i)+1<<" " << faces(1,i)+1 << " " << faces(2,i)+1 << std::endl;;
    }
    out.close();
}

template<typename Type,int _Options>
void EigenFunctions<Type,_Options>::saveEigenPoints(Eigen::Matrix<Type, 3, Eigen::Dynamic> points, Eigen::Matrix4Xi polygons, Eigen::Matrix4Xi colors, std::string savePath)
{
    std::ofstream out(savePath/*, std::ios::trunc*/);
    if(!out){
        std::cerr<<"write file:"<<savePath<<" fail!"<<std::endl<<std::flush;
        return;
    }
    if(points.size()==0){
        std::cerr<<"points is empty!"<<std::endl<<std::flush;
        return;
    }

    bool hasColor=colors.size()>0;
    for (int i = 0; i < points.cols(); i++) {
        //std::cout<<i<<std::endl<<std::flush;
        out << "v " << points(0,i)<<" " << points(1,i) << " " << points(2,i) << " ";
        if(hasColor){
            out << colors(0,i)<<" " << colors(1,i) << " " << colors(2,i) << " "<< colors(3,i);
        }
        out<< std::endl;
    }
    //std::cout<<"------------"<<std::endl<<std::flush;
    for(int i=0;i<polygons.cols();i++){
        //std::cout<<i<<std::endl<<std::flush;
        out << "f " << polygons(0,i)+1<<" " << polygons(1,i)+1 << " " << polygons(2,i)+1 <<" "<<polygons(3,i)+1 << std::endl;;
    }
    out.close();
}

template<typename Type, int _Options>
void EigenFunctions<Type,_Options>::saveEigenPoints(std::vector<Type> &points, std::vector<int> faces, std::string savePath)
{
    std::ofstream out(savePath/*, std::ios::trunc*/);
    if(!out){
        std::cerr<<"write file:"<<savePath<<" fail!"<<std::endl<<std::flush;
        return;
    }
    if(points.size()==0){
        std::cerr<<"points is empty!"<<std::endl<<std::flush;
        return;
    }
    //std::cout<<"***********"<<std::endl<<std::flush;
    for (int i = 0; i < points.size()/3; i++) {
        //        if(std::isnan(points[i*3+0])||std::isnan(points[i*3+1]||std::isnan(points[i*3+0]))){
        //            std::cerr << "v " << points[i*3+0]<<" " << points[i*3+1] << " " << points[i*3+2]<<std::endl;
        //        }
        out << "v " << points[i*3+0]<<" " << points[i*3+1] << " " << points[i*3+2];
        out<< std::endl;
    }
    //std::cout<<"------------"<<std::endl<<std::flush;
    for(int i=0;i<faces.size()/3;i++){
        out << "f " << faces[i*3+0]+1<<" " << faces[i*3+1]+1 << " " << faces[i*3+2]+1 << std::endl;;
    }
    out.close();
    //std::cout<<"++++++++++"<<std::endl<<std::flush;
}

template<typename Type, int _Options>
void EigenFunctions<Type,_Options>::saveEigenPointsWithColors(std::vector<Type> &points, std::vector<int> faces, std::vector<int> colors, std::string savePath)
{
    std::ofstream out(savePath/*, std::ios::trunc*/);
    if(!out){
        std::cerr<<"write file:"<<savePath<<" fail!"<<std::endl<<std::flush;
        return;
    }
    if(points.size()==0){
        std::cerr<<"points is empty!"<<std::endl<<std::flush;
        return;
    }

    for (int i = 0; i < points.size()/3; i++) {
        out << "v " << points[i*3+0]<<" " << points[i*3+1] << " " << points[i*3+2]<<" "
            <<colors[i*3+0]<<" " << colors[i*3+1] << " " << colors[i*3+2];
        out<< std::endl;
    }
    //std::cout<<"------------"<<std::endl<<std::flush;
    for(int i=0;i<faces.size()/3;i++){
        out << "f " << faces[i*3+0]+1<<" " << faces[i*3+1]+1 << " " << faces[i*3+2]+1 << std::endl;;
    }
    out.close();
}

template<typename Type, int _Options>
void EigenFunctions<Type,_Options>::saveEigenPointsWithKey(std::vector<Type> &points, std::vector<int> faces, std::vector<int> keys, std::string savePath)
{
    std::ofstream out(savePath/*, std::ios::trunc*/);
    if(!out){
        std::cerr<<"write file:"<<savePath<<" fail!"<<std::endl<<std::flush;
        return;
    }
    if(points.size()==0){
        std::cerr<<"points is empty!"<<std::endl<<std::flush;
        return;
    }
    std::map<int,int>keymap;
    for (int i = 0; i < keys.size(); i++) {
        keymap[keys[i]]=keys[i];
    }
    for (int i = 0; i < points.size()/3; i++) {
        out << "v " << points[i*3+0]<<" " << points[i*3+1] << " " << points[i*3+2] << " ";
        if(keymap.count(i)){
            out << 1<<" " << 0 << " " << 0 << " "<< 0;
        }else{
            out << 1<<" " << 1 << " " << 1 << " "<< 0;
        }
        out<< std::endl;
    }
    //std::cout<<"------------"<<std::endl<<std::flush;
    for(int i=0;i<faces.size()/3;i++){
        out << "f " << faces[i*3+0]+1<<" " << faces[i*3+1]+1 << " " << faces[i*3+2]+1 << std::endl;;
    }
    out.close();
}

template<typename Type, int _Options>
void EigenFunctions<Type,_Options>::saveEigenPointsWithUV(std::vector<Type> &points, std::vector<int> faces, std::vector<Type> uvs, std::string savePath)
{
    std::ofstream out(savePath/*, std::ios::trunc*/);
    if(!out){
        std::cerr<<"write file:"<<savePath<<" fail!"<<std::endl<<std::flush;
        return;
    }
    if(points.size()==0){
        std::cerr<<"points is empty!"<<std::endl<<std::flush;
        return;
    }

    for (int i = 0; i < points.size()/3; i++) {
        out << "v " << points[i*3+0]<<" " << points[i*3+1] << " " << points[i*3+2];
        out<< std::endl;
    }
    for (int i = 0; i < uvs.size()/2; i++) {
        out << "vt " << uvs[i*2+0]<<" " << uvs[i*2+1];
        out<< std::endl;
    }
    //std::cout<<"------------"<<std::endl<<std::flush;
    for(int i=0;i<faces.size()/3;i++){
        out << "f " << faces[i*3+0]+1<<" " << faces[i*3+1]+1 << " " << faces[i*3+2]+1 << std::endl;;
    }
    out.close();
}

template<typename Type, int _Options>
void EigenFunctions<Type,_Options>::saveEigenPointsWithKey(Eigen::Matrix<Type, 3, Eigen::Dynamic> &points, Eigen::Matrix4Xi polygons, std::vector<int> keys, std::string savePath)
{
    std::ofstream out(savePath/*, std::ios::trunc*/);
    if(!out){
        std::cerr<<"write file:"<<savePath<<" fail!"<<std::endl<<std::flush;
        return;
    }
    if(points.size()==0){
        std::cerr<<"points is empty!"<<std::endl<<std::flush;
        return;
    }
    std::map<int,int>keymap;
    for (int i = 0; i < keys.size(); i++) {
        keymap[keys[i]]=keys[i];
    }
    for (int i = 0; i < points.cols(); i++) {
        //std::cout<<i<<std::endl<<std::flush;
        out << "v " << points(0,i)<<" " << points(1,i) << " " << points(2,i) << " ";
        if(keymap.count(i)){
            out << 1<<" " << 0 << " " << 0 << " "<< 0;
        }else{
            out << 1<<" " << 1 << " " << 1 << " "<< 0;
        }
        out<< std::endl;
    }
    //std::cout<<"------------"<<std::endl<<std::flush;
    for(int i=0;i<polygons.cols();i++){
        std::cout<<i<<std::endl<<std::flush;
        out << "f " << polygons(0,i)+1<<" " << polygons(1,i)+1 << " " << polygons(2,i)+1 <<" "<<polygons(3,i)+1 << std::endl;;
    }
    out.close();
}

template<typename Type, int _Options>
void EigenFunctions<Type,_Options>::saveEigenPointsWithWeights(Eigen::Matrix<Type, 3, Eigen::Dynamic> &points, Eigen::Matrix3Xi faces, std::vector<float> weights, std::string savePath)
{
    std::ofstream out(savePath/*, std::ios::trunc*/);
    if(!out){
        std::cerr<<"write file:"<<savePath<<" fail!"<<std::endl<<std::flush;
        return;
    }
    if(points.size()==0){
        std::cerr<<"points is empty!"<<std::endl<<std::flush;
        return;
    }
    std::map<int,int>keymap;

    for (int i = 0; i < points.cols(); i++) {
        out << "v " << points(0,i)<<" " << points(1,i) << " " << points(2,i) << " ";
        if(weights[i]==1.0){
            out << 1<<" " << 1 << " " << 1 << " "<< 0;
        }else{

            if(weights[i]<0){
                out << -weights[i]  <<" " << 0<< " " << 0 << " "<< 0;
            }else if(weights[i]<0.5){
                out << 0<<" " << (1-weights[i]) << " " << 0 << " "<< 0;
            }else if(weights[i]<0.75){
                out << 1<<" " << 0.5 << " " <<1-weights[i] << " "<< 0;
            }else{
                out << 0<<" " << 0 << " " << weights[i] << " "<< 0;
            }

        }
        out<< std::endl;
    }
    //std::cout<<"------------"<<std::endl<<std::flush;
    for(int i=0;i<faces.cols();i++){
        out << "f " << faces(0,i)+1<<" " << faces(1,i)+1 << " " << faces(2,i)+1 << std::endl;;
    }
    out.close();
}

template<typename Type, int _Options>
void EigenFunctions<Type,_Options>::saveEigenPointsWithKey(Eigen::Matrix<Type, 3, Eigen::Dynamic> &points, Eigen::Matrix3Xi faces, std::vector<int> keys, std::string savePath)
{
    std::ofstream out(savePath/*, std::ios::trunc*/);
    if(!out){
        std::cerr<<"write file:"<<savePath<<" fail!"<<std::endl<<std::flush;
        return;
    }
    if(points.size()==0){
        std::cerr<<"points is empty!"<<std::endl<<std::flush;
        return;
    }
    std::map<int,int>keymap;
    for (int i = 0; i < keys.size(); i++) {
        keymap[keys[i]]=keys[i];
    }
    for (int i = 0; i < points.cols(); i++) {
        out << "v " << points(0,i)<<" " << points(1,i) << " " << points(2,i) << " ";
        if(keymap.count(i)){
            out << 1<<" " << 0 << " " << 0 << " "<< 0;
        }else{
            out << 1<<" " << 1 << " " << 1 << " "<< 0;
        }
        out<< std::endl;
    }
    //std::cout<<"------------"<<std::endl<<std::flush;
    for(int i=0;i<faces.cols();i++){
        out << "f " << faces(0,i)+1<<" " << faces(1,i)+1 << " " << faces(2,i)+1 << std::endl;;
    }
    out.close();
}

template<typename Type,int _Options>
Type EigenFunctions<Type,_Options>::computeScale(Eigen::Matrix<Type, 3, Eigen::Dynamic> points)
{
    int vnum=points.cols();
    Type maxX=1e-5;
    Type maxY=1e-5;
    Type maxZ=1e-5;
    Type minX=1e5;
    Type minY=1e5;
    Type minZ=1e5;
    for (int i=0;i<vnum;i++) {
        Type x = points(0,i),y = points(1,i),z = points(2,i);
        maxX = std::fmax(x ,maxX ); maxY = std::fmax(y ,maxY ) ; maxZ = std::fmax(z ,maxZ );
        minX = std::fmin(x ,minX ); minY = std::fmin(y ,minY ) ; minZ = std::fmin(z ,minZ );
    }
    Type xlen,ylen,zlen;
    xlen = fabs(maxX-minX);
    ylen = fabs(maxY-minY);
    zlen = fabs(maxZ-minZ);
    Eigen::Vector3f meshScale(xlen,ylen,zlen);
    return meshScale.maxCoeff();
}

template<typename Type,int _Options>
void EigenFunctions<Type,_Options>::center(Eigen::Matrix<Type, 3, Eigen::Dynamic> &points)
{
    Eigen::Matrix<Type, Eigen::Dynamic, 1> c=points.rowwise().mean();
    points.colwise()-=c;
}
template<typename Type,int _Options>
void EigenFunctions<Type,_Options>::polygons2faces(const Eigen::Matrix<Type, 4, Eigen::Dynamic> &polygons, Eigen::Matrix<Type, 3, Eigen::Dynamic> &faces)
{
    faces.resize(3,polygons.cols()*2);
    for(int i=0;i<polygons.cols();i++){
        int i0=polygons(0,i);
        int i1=polygons(1,i);
        int i2=polygons(2,i);
        int i3=polygons(3,i);
        faces(0,2*i)=i0;
        faces(1,2*i)=i1;
        faces(2,2*i)=i2;

        faces(0,2*i+1)=i0;
        faces(1,2*i+1)=i2;
        faces(2,2*i+1)=i3;
    }
}
template<typename Type,int _Options>
Eigen::Matrix3Xf EigenFunctions<Type,_Options>::computeFaceNormals(Eigen::Matrix3Xf points, Eigen::Matrix3Xi faces)
{
    Eigen::Matrix3Xf faceNormals;
    faceNormals.resize(3,faces.cols());
    for(int i=0;i<faces.cols();i++){
        int i0=faces(0,i);
        int i1=faces(1,i);
        int i2=faces(2,i);
        Eigen::Vector3f v1Subv0=points.col(i1)-points.col(i0);
        Eigen::Vector3f v2Subv0=points.col(i2)-points.col(i0);
        Eigen::Vector3f normal=v1Subv0.cross(v2Subv0);
        faceNormals.col(i)=normal;
    }
    return faceNormals;
}

template<typename Type,int _Options>
Eigen::SparseMatrix<float> EigenFunctions<Type,_Options>::computeFaceNormalsWeight(int64_t vnum, Eigen::Matrix3Xi faces)
{
    int64_t fnum=faces.cols();
    Eigen::SparseMatrix<float> weights;
    weights.resize(vnum,fnum);
    std::vector<Eigen::Triplet<float>> trips;
    trips.resize(fnum*3);
    for(int64_t i=0;i<fnum;i++){
        int r0=faces(0,i);
        int r1=faces(1,i);
        int r2=faces(2,i);
        trips[3*i]=Eigen::Triplet<float>(r0,i,1.0);
        trips[3*i+1]=Eigen::Triplet<float>(r1,i,1.0);
        trips[3*i+2]=Eigen::Triplet<float>(r2,i,1.0);
    }
    weights.setFromTriplets(trips.begin(),trips.end());//vnumxfnum
    return std::move(weights);
}

template<typename Type,int _Options>
Eigen::Matrix3Xf EigenFunctions<Type,_Options>::computeVertNormals(Eigen::Matrix3Xf points, Eigen::Matrix3Xi faces)
{
    int64_t vnum=points.cols();
    Eigen::SparseMatrix<float> weights=computeFaceNormalsWeight(vnum,faces);//vnumxfnum
    Eigen::Matrix3Xf fn=computeFaceNormals(points,faces);//3xfnum
    Eigen::MatrixX3f vn=weights*fn.transpose();//vnumxfnum * fnumx3=vnumx3
    for(int k=0;k<vn.rows();k++){
        float tmp=vn.row(k).norm();
        vn.row(k)/=tmp;
    }
    return  vn.transpose();
}

template<typename Type, int _Options>
Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic, _Options> EigenFunctions<Type,_Options>::vector2Matrix(const std::vector<Type> &vec, int rows, int cols)
{
    return Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic,_Options> >(vec.data(),rows,cols);
}

template<typename Type, int _Options>
std::tuple<Eigen::Matrix<Type, 3, Eigen::Dynamic>, Eigen::Matrix3Xi> EigenFunctions<Type,_Options>::mergeObjs(std::vector<Eigen::Matrix<Type, 3, Eigen::Dynamic> > &objs, std::vector<Eigen::Matrix3Xi> &faces)
{
    int allColsOfObjs=0;
    int allColsOfFaces=0;
    for(int k=0;k<objs.size();k++){
        allColsOfObjs+=objs[k].cols();
        allColsOfFaces+=faces[k].cols();
    }
    Eigen::Matrix<Type, 3, Eigen::Dynamic> allPoints;
    allPoints.resize(3,allColsOfObjs);
    Eigen::Matrix3Xi allFaces;
    allFaces.resize(3,allColsOfFaces);
    int offset1=0;
    int offset2=0;
    for(int k=0;k<objs.size();k++){
        allPoints.block(0,offset1,3,objs[k].cols())=objs[k];
        faces[k].array()+=offset1;
        allFaces.block(0,offset2,3,faces[k].cols())=faces[k];
        offset1+=objs[k].cols();
        offset2+=faces[k].cols();
    }
    return std::make_tuple(allPoints,allFaces);
}

//template<typename Type, int _Options>
//Eigen::Quaternion<Type> EigenFunctions<Type,_Options>::Slerp(Eigen::Quaternion<Type> &v0, Eigen::Quaternion<Type> &v1, Type t)
//{
//    // Only unit quaternions are valid rotations.
//    // Normalize to avoid undefined behavior.
//    v0.normalize();
//    v1.normalize();

//    // Compute the cosine of the angle between the two vectors.
//    Type dot = v0.dot(v1);

//    // If the dot product is negative, slerp won't take
//    // the shorter path. Note that v1 and -v1 are equivalent when
//    // the negation is applied to all four components. Fix by
//    // reversing one quaternion.
//    if (dot < 0.0f) {
//        v1=-v1;
//        dot = -dot;
//    }
//    const Type DOT_THRESHOLD = 0.9995;
//    if (dot > DOT_THRESHOLD) {
//        // If the inputs are too close for comfort, linearly interpolate
//        // and normalize the result.

//        Eigen::Quaternion<Type> result = v0 + t*(v1 - v0);
//        result.normalize();
//        return result;
//    }

//    // Since dot is in range [0, DOT_THRESHOLD], acos is safe
//    double theta_0 = acos(dot);        // theta_0 = angle between input vectors
//    double theta = theta_0*t;          // theta = angle between v0 and result
//    double sin_theta = sin(theta);     // compute this value only once
//    double sin_theta_0 = sin(theta_0); // compute this value only once

//    double s0 = cos(theta) - dot * sin_theta / sin_theta_0;  // == sin(theta_0 - theta) / sin(theta_0)
//    double s1 = sin_theta / sin_theta_0;

//    return (s0 * v0) + (s1 * v1);
//}


#endif // EIGENFUNCTIONS_H
