#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <Eigen/Dense>
namespace Rigid{
template <typename Type>
class Transform
{
public:
    Transform(){rotation=Eigen::Matrix<Type, 3, 3>::Identity();translation=Eigen::Matrix<Type, 3, 1>::Zero();}
    Transform(Eigen::Matrix<Type, 3, 3> R){rotation=R;translation=Eigen::Matrix<Type, 3, 1>::Zero();}
    Transform(Eigen::Matrix<Type, 3, 1> t){rotation=Eigen::Matrix<Type, 3, 3>::Identity();translation=t;}
    Transform(Eigen::Matrix<Type, 3, 3> R,Eigen::Matrix<Type, 3, 1> t){rotation=R;translation=t;}
    Eigen::Matrix<Type, 3, 3> getRotation() const;
    void setRotation(const Eigen::Matrix<Type, 3, 3> &value);
    void applyRotation(Eigen::Matrix<Type, 3, Eigen::Dynamic>& points);
    void applyTranslation(Eigen::Matrix<Type, 3, Eigen::Dynamic>& points);
    void apply(Eigen::Matrix<Type, 3, Eigen::Dynamic>& points);
    Eigen::Matrix<Type, 3, 1> getTranslation() const;
    void setTranslation(const Eigen::Matrix<Type, 3, 1> &value);
    Transform<Type> operator*(Transform<Type> other){
        Transform<Type> transform;
        Eigen::Matrix<Type, 3, 3> R=this->rotation*other.rotation;
        Eigen::Matrix<Type, 3, 1> t=this->rotation*other.translation+this->translation;
        transform.setRotation(R);
        transform.setTranslation(t);
        return transform;
    }
    friend std::ostream& operator<< (std::ostream &o,const Transform<Type>& t){
        o<<"transform:"<<std::endl;
       return o<<t.rotation<<std::endl<<t.translation;
    }

private:
    Eigen::Matrix<Type, 3, 3> rotation;
    Eigen::Matrix<Type, 3, 1> translation;

};
template <typename Type>
Eigen::Matrix<Type, 3, 3> Transform<Type>::getRotation() const
{
    return rotation;
}
template <typename Type>
void Transform<Type>::setRotation(const Eigen::Matrix<Type, 3, 3> &value)
{
    rotation = value;
}

template<typename Type>
void Transform<Type>::applyRotation(Eigen::Matrix<Type, 3, Eigen::Dynamic> &points)
{
    points=rotation*points;
}
template<typename Type>
void Transform<Type>::applyTranslation(Eigen::Matrix<Type, 3, Eigen::Dynamic> &points)
{
    points.colwise()+=translation;
}
template<typename Type>
void Transform<Type>::apply(Eigen::Matrix<Type, 3, Eigen::Dynamic>& points)
{
    points=rotation*points;
    points.colwise()+=translation;
}
template <typename Type>
Eigen::Matrix<Type, 3, 1> Transform<Type>::getTranslation() const
{
return translation;
}
template <typename Type>
void Transform<Type>::setTranslation(const Eigen::Matrix<Type, 3, 1> &value)
{
translation = value;
}
}


#endif // TRANSFORM_H
