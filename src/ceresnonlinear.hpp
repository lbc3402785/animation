#ifndef CERESNONLINEAR_HPP
#define CERESNONLINEAR_HPP
#include <map>
#include <Eigen/Dense>
#include "facemodel.h"
namespace fitting {
struct LandmarkCost{
    LandmarkCost(FaceModel keyModel,float  fx,int vertexId,Eigen::Vector3f dstKeyPoint):keyModel(keyModel),fx(fx),vertexId(vertexId),dstKeyPoint(dstKeyPoint){

    }
    template<typename T>
    bool operator()(const T* const quat, const T* const translation,const T* const shape_coeffs, const T* const blendshape_coeffs,T* residual)const;

    //bool operator()(const double* const quat, const double* const translation,double* residual)const;
private:
    FaceModel keyModel;
    int vertexId;
    Eigen::Vector3f dstKeyPoint;

    float fx;
};
template<typename T>
std::array<T, 3> get_shape_point(FaceModel keyModel,int vertId,const T* const shape_coeffs, const T* const blendshape_coeffs){
    int num_coeffs_fitting=keyModel.SB.cols();
    int num_blendshapes=keyModel.EB.cols();
    auto mean=keyModel.Face.row(vertId);//1x3
    auto basis=keyModel.SB.block(3*vertId,0,3,num_coeffs_fitting);
    auto blendShapes=keyModel.EB.block(3*vertId,0,3,num_blendshapes);
    std::array<T, 3> point{T(mean(0,0)), T(mean(0,1)), T(mean(0,2))};

    for (int i = 0; i < num_coeffs_fitting; ++i)
    {
        point[0] += T(basis.row(0).col(i)(0)) * shape_coeffs[i]; // it seems to be ~15% faster when these are
                                                                 // static_cast<double>() instead of T()?
    }
    for (int i = 0; i < num_coeffs_fitting; ++i)
    {
        point[1] += T(basis.row(1).col(i)(0)) * shape_coeffs[i];
    }
    for (int i = 0; i < num_coeffs_fitting; ++i)
    {
        point[2] += T(basis.row(2).col(i)(0)) * shape_coeffs[i];
    }


    for (int i = 0; i < num_blendshapes; ++i)
    {
        point[0] += T(blendShapes.row(0).col(i)(0)) * blendshape_coeffs[i]; // it seems to be ~15% faster when these are
                                                                 // static_cast<double>() instead of T()?
    }
    for (int i = 0; i < num_blendshapes; ++i)
    {
        point[1] += T(blendShapes.row(1).col(i)(0)) * blendshape_coeffs[i];
    }
    for (int i = 0; i < num_blendshapes; ++i)
    {
        point[2] += T(blendShapes.row(2).col(i)(0)) * blendshape_coeffs[i];
    }
    return point;
}
template<typename T>
bool LandmarkCost::operator()(const T* const quat, const T* const translation,const T* const shape_coeffs, const T* const blendshape_coeffs, T* residual)const{
    Eigen::Quaternion<T> q(quat[0],quat[1],quat[2],quat[3]);
    Eigen::Matrix<T,3,3> Rotation=q.toRotationMatrix();
    Eigen::Matrix<T, 3, 1> t(translation[0],translation[1],translation[2]);

    const auto point_arr=get_shape_point<T>(keyModel, vertexId, shape_coeffs, blendshape_coeffs);
    Eigen::Matrix<T,3,1> tmpSrcKeyPoint;
    tmpSrcKeyPoint(0,0)=point_arr[0];
    tmpSrcKeyPoint(1,0)=point_arr[1];
    tmpSrcKeyPoint(2,0)=point_arr[2];
    tmpSrcKeyPoint=Rotation*tmpSrcKeyPoint+t;

    T dis=tmpSrcKeyPoint(2,0);
    Eigen::Matrix<T,3,1> tmpDstKeyPoint;
    tmpDstKeyPoint(0,0)=T(dstKeyPoint(0,0));
    tmpDstKeyPoint(1,0)=T(dstKeyPoint(1,0));
    tmpDstKeyPoint(2,0)=T(dstKeyPoint(2,0));
    Eigen::Matrix<T,3,1> diff=tmpDstKeyPoint/T(fx)-tmpSrcKeyPoint/dis;
    residual[0]=T(diff(0,0));
    residual[1]=T(diff(1,0));

    return true;
}
struct Key2DPointCost{
    Key2DPointCost(float  fx,Eigen::Vector3f srcKeyPoint,Eigen::Vector3f dstKeyPoint):fx(fx),srcKeyPoint(srcKeyPoint),dstKeyPoint(dstKeyPoint){
        weight=1.0f;
    }
    template<typename T>
    bool operator()(const T* const quat, const T* const translation,T* residual)const;
    float weight=1.0f;
    //bool operator()(const double* const quat, const double* const translation,double* residual)const;
private:
    Eigen::Vector3f srcKeyPoint;
    Eigen::Vector3f dstKeyPoint;
    float fx;
};
template<typename T>
bool Key2DPointCost::operator()(const T* const quat, const T* const translation, T* residual)const{
    Eigen::Quaternion<T> q(quat[0],quat[1],quat[2],quat[3]);
    Eigen::Matrix<T,3,3> Rotation=q.toRotationMatrix();
    Eigen::Matrix<T, 3, 1> t(translation[0],translation[1],translation[2]);

    Eigen::Matrix<T,3,1> tmpSrcKeyPoint;
    tmpSrcKeyPoint(0,0)=T(srcKeyPoint(0,0));
    tmpSrcKeyPoint(1,0)=T(srcKeyPoint(1,0));
    tmpSrcKeyPoint(2,0)=T(srcKeyPoint(2,0));
    tmpSrcKeyPoint=Rotation*tmpSrcKeyPoint+t;

    T dis=tmpSrcKeyPoint(2,0);
    Eigen::Matrix<T,3,1> tmpDstKeyPoint;
    tmpDstKeyPoint(0,0)=T(dstKeyPoint(0,0));
    tmpDstKeyPoint(1,0)=T(dstKeyPoint(1,0));
    tmpDstKeyPoint(2,0)=T(dstKeyPoint(2,0));
//    Eigen::Matrix<T,3,1> diff=dis*tmpDstKeyPoint-T(fx)*tmpSrcKeyPoint;
    Eigen::Matrix<T,3,1> diff=tmpDstKeyPoint/T(fx)-tmpSrcKeyPoint/dis;
    diff*=(T)weight;
    residual[0]=T(diff(0,0));
    residual[1]=T(diff(1,0));
    return true;
}

struct Key3DPointCost{
    Key3DPointCost(FaceModel FMFull,int vertexId,Eigen::Vector3f keyPoint):FMFull(FMFull),vertexId(vertexId),srcKeyPoint(keyPoint){

    }
    bool operator()(const double* const quat, const double* const translation,
                    const double* const scale,const double* const shapeCoeffs, double* residual)const;
private:
    FaceModel FMFull;
    int vertexId;
    Eigen::Vector3f srcKeyPoint;
};

bool Key3DPointCost::operator()(const double* const quat, const double* const translation,
                                const double* const scale,const double* const shapeCoeffs, double* residual)const{
    Eigen::Quaternion<double> q(quat[0],quat[1],quat[2],quat[3]);
    Eigen::Matrix<double,3,3> Rotation=q.toRotationMatrix();
    Eigen::Matrix<double, 3, 1> t(translation[0],translation[1],translation[2]);
    Eigen::Matrix<float,3,3> Rf=Rotation.cast<float>();
    Eigen::Matrix<float, 3, 1> tf=t.cast<float>();
    float tmpScale=(float)scale[0];
    Eigen::Matrix<double, Dynamic, Dynamic, RowMajor> shapeX;
    shapeX.resize(FMFull.SB.cols(),1);
    memcpy(shapeX.data(),shapeCoeffs,sizeof(double)*FMFull.SB.cols());
    Eigen::Vector3f tmpKeyPoint=srcKeyPoint*tmpScale;
    tmpKeyPoint=Rf*tmpKeyPoint+tf;


    MatF SX=shapeX.cast<float>();
    MatF FaceS = FMFull.SB * SX;
    MatF S = Reshape(FaceS, 3);
    MatF curFace=FMFull.Face + S;
    Eigen::Matrix3Xf variDstPoints=curFace.transpose();
    Eigen::Vector3f dstKeyPoint=variDstPoints.col(vertexId);
    Eigen::Vector3f diff=tmpKeyPoint-dstKeyPoint;
    //std::cout<<"diff:"<<diff<<std::endl<<std::flush;
    residual[0]=double(diff[0]);
    residual[1]=double(diff[1]);
    residual[2]=double(diff[2]);
    return true;
}
std::map<int,int> pointsMap;
Eigen::Matrix3Xf srcPoints;
FaceModel FMFull;
bool FIXFIRSTSHAPE = false;
struct Closest3DShapePointCost{
public:
    Closest3DShapePointCost(int srcVertexId):srcVertexId(srcVertexId){
        weight=1.0f;
    }
    template <typename T>
    bool operator()(const T* const quat, const T* const translation,const T* const shapeCoeffs, T* residual)const;
    float getWeight() const;
    void setWeight(float value);

private:
    float weight;
    int srcVertexId;

};
template <typename T>
bool Closest3DShapePointCost::operator()(const T* const quat, const T* const translation,const T* const shapeCoeffs, T* residual)const{
    std::vector<double> qtmp(4,0.0);
    memcpy(qtmp.data(),quat,sizeof(double)*4);
    Eigen::Quaternion<double> q(qtmp[0],qtmp[1],qtmp[2],qtmp[3]);
    Eigen::Matrix<double,3,3> Rotation=q.toRotationMatrix();
    std::vector<double> ttmp(3,0.0);
    memcpy(ttmp.data(),translation,sizeof(double)*3);
    Eigen::Matrix<double, 3, 1> t(ttmp[0],ttmp[1],ttmp[2]);
    Eigen::Matrix<float,3,3> Rf=Rotation.cast<float>();
    Eigen::Matrix<float, 3, 1> tf=t.cast<float>();
    Eigen::Matrix<double, Dynamic, Dynamic, RowMajor> shapeX;
    shapeX.resize(FMFull.SB.cols(),1);
    memcpy(shapeX.data(),shapeCoeffs,sizeof(double)*FMFull.SB.cols());
    MatF SX=shapeX.cast<float>();
    MatF FaceS = FMFull.SB * SX;
    MatF S = Reshape(FaceS, 3);
    MatF curFace=FMFull.Face + S;
    Eigen::Matrix3Xf variDstPoints=curFace.transpose();
    Eigen::Vector3f srcPoint=srcPoints.col(srcVertexId);
    int dstVertexId=pointsMap[srcVertexId];
    Eigen::Vector3f dstPoint=variDstPoints.col(dstVertexId);
    Eigen::Vector3f diff=(Rf*srcPoint+tf-dstPoint)*weight;
    residual[0]=T(diff[0]);
    residual[1]=T(diff[1]);
    residual[2]=T(diff[2]);
    return true;
}
struct Closest3DPointCost{
public:
    Closest3DPointCost(int srcVertexId):srcVertexId(srcVertexId){
        weight=1.0f;
        isKey=false;
    }
    template <typename T>
    bool operator()(const T* const scale,const T* const quat, const T* const translation,const T* const shapeCoeffs,const T* const expressCoeffs, T* residual)const;

    float getWeight() const;

    void setWeight(float value);

    bool getIsKey() const;
    void setIsKey(bool value);

    Eigen::Matrix3Xi getFaces() const;
    void setFaces(const Eigen::Matrix3Xi &value);

private:
    int srcVertexId;
    float weight;
    bool isKey;
    Eigen::Matrix3Xi faces;
};

float Closest3DPointCost::getWeight() const
{
    return weight;
}

void Closest3DPointCost::setWeight(float value)
{
    weight = value;
}

bool Closest3DPointCost::getIsKey() const
{
    return isKey;
}

void Closest3DPointCost::setIsKey(bool value)
{
    isKey = value;
}

Eigen::Matrix3Xi Closest3DPointCost::getFaces() const
{
    return faces;
}

void Closest3DPointCost::setFaces(const Eigen::Matrix3Xi &value)
{
    faces = value;
}

template<typename T>
bool Closest3DPointCost::operator()(const T* const scaleptr,const T* const quatptr, const T* const translationptr,const T* const shapeCoeffs,const T * const expressCoeffs, T *residual) const
{
    constexpr int numOfShapeCoefs=199;
    constexpr int numOfExpressCoeffs=100;
    int dstVertexId=pointsMap[srcVertexId];
    T scale(scaleptr[0]);
    const Eigen::Quaternion<T>  quat(quatptr[0],quatptr[1],quatptr[2],quatptr[3]);
    const Eigen::Matrix<T,3,1>  translation(translationptr[0],translationptr[1],translationptr[2]);
    Eigen::Vector3f mean = FMFull.Face.row(dstVertexId);
    MatF shapeBasis=FMFull.SB.block(3*dstVertexId,0,3,numOfShapeCoefs);
    MatF expressBasis=FMFull.EB.block(3*dstVertexId,0,3,numOfExpressCoeffs);
    Eigen::Matrix<T,3,1> dstPoint;
    dstPoint(0)=T(mean[0]);
    dstPoint(1)=T(mean[1]);
    dstPoint(2)=T(mean[2]);
    for (int i = 0; i < numOfShapeCoefs; ++i)
    {
        if(FIXFIRSTSHAPE&&i==0)continue;
        dstPoint(0) += T(shapeBasis.row(0).col(i)(0)) * shapeCoeffs[i]; // it seems to be ~15% faster when these are
        // static_cast<double>() instead of T()?
    }
    for (int i = 0; i < numOfShapeCoefs; ++i)
    {
        if(FIXFIRSTSHAPE&&i==0)continue;
        dstPoint(1) += T(shapeBasis.row(1).col(i)(0)) * shapeCoeffs[i];
    }
    for (int i = 0; i < numOfShapeCoefs; ++i)
    {
        if(FIXFIRSTSHAPE&&i==0)continue;
        dstPoint(2) += T(shapeBasis.row(2).col(i)(0)) * shapeCoeffs[i];
    }


    for (int i = 0; i < numOfExpressCoeffs; ++i)
    {
        dstPoint(0) += T(expressBasis.row(0).col(i)(0)) * expressCoeffs[i];
    }
    for (int i = 0; i < numOfExpressCoeffs; ++i)
    {
        dstPoint(1) += T(expressBasis.row(1).col(i)(0)) * expressCoeffs[i];
    }
    for (int i = 0; i < numOfExpressCoeffs; ++i)
    {
        dstPoint(2) += T(expressBasis.row(2).col(i)(0)) * expressCoeffs[i];
    }


    Eigen::Matrix<T,3,3> Rotation=quat.toRotationMatrix();
    Eigen::Matrix<T,3,1> tmp;
    tmp(0)=T(srcPoints.col(srcVertexId)[0]);
    tmp(1)=T(srcPoints.col(srcVertexId)[1]);
    tmp(2)=T(srcPoints.col(srcVertexId)[2]);
    Eigen::Matrix<T,3,1> srcPoint=(Rotation*tmp+translation)*scale;
    Eigen::Matrix<T,3,1> diff=(srcPoint-dstPoint)*T(weight);
    //    if(isKey){
    residual[0]=diff[0];
    residual[1]=diff[1];
    residual[2]=diff[2];
    //        residual[0]=T(diff.norm());
    //    }else{
    //        residual[0]=T(GeometryFunctions<Kernel,float>::calSingleDistance(srcPoint,variDstPoints,faces));
    //    }

    return true;
}

float Closest3DShapePointCost::getWeight() const
{
    return weight;
}

void Closest3DShapePointCost::setWeight(float value)
{
    weight = value;
}
/**
 * Cost function for a prior on the parameters.
 *
 * Prior towards zero (0, 0...) for the parameters.
 * Note: The weight is inside the norm, so may not correspond to the "usual"
 * formulas. However I think it's equivalent up to a scaling factor, but it
 * should be checked.
 */
struct PriorCost
{

    /**
     * Creates a new prior object with set number of variables and a weight.
     *
     * @param[in] num_variables Number of variables that the parameter vector contains.
     * @param[in] weight A weight that the parameters are multiplied with.
     */
    PriorCost(int numVariables, double weight = 1.0) : numVariables(numVariables), weight(weight){};

    /**
     * Cost function implementation.
     *
     * @param[in] x An array of parameters.
     * @param[in] residual An array of the resulting residuals.
     * @return Returns true. The ceres documentation is not clear about that I think.
     */
    template <typename T>
    bool operator()(const T* const x, T* residual) const
    {
        for (int i = 0; i < numVariables; ++i)
        {
            residual[i] = weight * x[i];
        }
        return true;
    };


public:
    double PriorCost::getWeight() const
    {
        return weight;
    }

    void PriorCost::setWeight(double value)
    {
        weight = value;
    }

private:
    int numVariables;
    double weight;
};
/**
 * Cost function for a prior on the parameters.
 *
 * Prior towards zero (0, 0...) for the parameters.
 * Note: The weight is inside the norm, so may not correspond to the "usual"
 * formulas. However I think it's equivalent up to a scaling factor, but it
 * should be checked.
 */
struct PriorCost2
{

    /**
     * Creates a new prior object with set number of variables and a weight.
     *
     * @param[in] num_variables Number of variables that the parameter vector contains.
     * @param[in] weight A weight that the parameters are multiplied with.
     */
    PriorCost2(int numVariables, double weight = 1.0) : numVariables(numVariables), weight(weight){
        variance=MatF::Ones(numVariables,1);
    };

    /**
     * Cost function implementation.
     *
     * @param[in] x An array of parameters.
     * @param[in] residual An array of the resulting residuals.
     * @return Returns true. The ceres documentation is not clear about that I think.
     */
    template <typename T>
    bool operator()(const T* const x, T* residual) const
    {
        for (int i = 0; i < numVariables; ++i)
        {
            double vi=(double)variance(i,0);
            residual[i]=weight*x[i]*x[i] /vi ;
        }
        return true;
    };


public:
    MatF variance;

private:
    int numVariables;
    double weight;
};
}
#endif // CERESNONLINEAR_HPP
