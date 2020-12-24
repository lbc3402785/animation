#ifndef DENSELAPLACE_H
#define DENSELAPLACE_H
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include<Eigen/SparseCholesky>
#include <vector>
#include <iostream>
#include <set>
#include "setoperator.h"
template <typename Type>
class DenseLaplace
{
private:
    Eigen::Matrix<Type, Eigen::Dynamic, 3,Eigen::RowMajor> points;

    Eigen::Matrix<int, Eigen::Dynamic, 3,Eigen::RowMajor> faces;
    Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic> L;
    Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic> Ls;
    Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic> A;
    Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic> AD;

    Eigen::LDLT<Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic>> LDLT;
    typedef typename  Eigen::Matrix<Type,Eigen::Dynamic, 1> VectorXT;
    VectorXT bx;
    VectorXT by;
    VectorXT bz;
    int pointsNum;
    int fixedAnchorsNum;//may be zero
    int targetsNum;
    int roiNum;
    std::map<int,int> roiRowNum;
    std::vector<int> roi;
    std::vector<int> targetIndexes;
    std::vector<int> fixedIndexes;
    std::pair<int,int> SortPair(int i,int j){
        if(i<j){
            return std::make_pair(i,j);
        }else{
            return std::make_pair(j,i);
        }
    }
    void init(const std::vector<Type>& vertices,const std::vector<int>& triangles,const std::vector<int>& targetIndexes,std::vector<int> fixedIndexes=std::vector<int>())
    {
        assert(vertices.size()%3==0);
        assert(triangles.size()%3==0);
        points.resize(vertices.size()/3,3);
        faces.resize(triangles.size()/3,3);
        memcpy(points.data(),vertices.data(),sizeof (Type)*vertices.size());
        memcpy(faces.data(),triangles.data(),sizeof(int)*triangles.size());
        pointsNum=points.rows();
        fixedAnchorsNum=fixedIndexes.size();
        targetsNum=targetIndexes.size();
        roiNum=roi.size();
        std::cout<<"pointsNum:"<<pointsNum<<std::endl;
        std::cout<<"fixedAnchorsNum:"<<fixedAnchorsNum<<std::endl;
        std::cout<<"targetsNum:"<<targetsNum<<std::endl;      
    }

    void updateCoordinate(){
        VectorXT vx=points.col(0);
        VectorXT vy=points.col(1);
        VectorXT vz=points.col(2);
        if(roiNum==0){
            bx=L*vx;
            by=L*vy;
            bz=L*vz;
            bx.conservativeResize(pointsNum +fixedAnchorsNum+ targetsNum);
            by.conservativeResize(pointsNum +fixedAnchorsNum+ targetsNum);
            bz.conservativeResize(pointsNum +fixedAnchorsNum+ targetsNum);
            //std::cout<<"5555"<<std::endl;
            // 用形变前坐标对固定锚点坐标进行赋值
            for (auto i = 0; i < fixedAnchorsNum; i++)
            {
                bx(pointsNum+i) = vx(fixedIndexes[i]);
                by(pointsNum+i) = vy(fixedIndexes[i]);
                bz(pointsNum+i) = vz(fixedIndexes[i]);
            }
        }else{
            VectorXT rvx=VectorXT::Zero(roiNum,1);
            VectorXT rvy=VectorXT::Zero(roiNum,1);
            VectorXT rvz=VectorXT::Zero(roiNum,1);
            for(int k=0;k<roi.size();k++){
                roiRowNum[roi[k]]=k;
                rvx[k]=vx(roi[k]);
                rvy[k]=vy(roi[k]);
                rvz[k]=vz(roi[k]);
            }
            bx=L*rvx;
            by=L*rvy;
            bz=L*rvz;
            bx.conservativeResize(roiNum +fixedAnchorsNum+ targetsNum);
            by.conservativeResize(roiNum +fixedAnchorsNum+ targetsNum);
            bz.conservativeResize(roiNum +fixedAnchorsNum+ targetsNum);
            //std::cout<<"5555"<<std::endl;
            // 用形变前坐标对固定锚点坐标进行赋值
            for (auto i = 0; i < fixedAnchorsNum; i++)
            {
                bx(roiNum+i) = vx(fixedIndexes[i]);
                by(roiNum+i) = vy(fixedIndexes[i]);
                bz(roiNum+i) = vz(fixedIndexes[i]);
            }
        }
    }
public:
    void computeLaplaceMarix(){
        VectorXT vx=points.col(0);
        VectorXT vy=points.col(1);
        VectorXT vz=points.col(2);
        std::set<std::pair<int,int>> edges;
        for(int64_t i=0;i<faces.rows();i++){
            int i0=faces(i,0);
            int i1=faces(i,1);
            int i2=faces(i,2);
            edges.insert(SortPair(i0,i1));
            edges.insert(SortPair(i1,i2));
            edges.insert(SortPair(i2,i0));
        }
        if(roiNum==0){
            L.resize(pointsNum,pointsNum);
            Ls.resize(pointsNum+fixedAnchorsNum+targetsNum,pointsNum);
            std::set<std::pair<int,int>>::iterator uvIter = edges.begin();
            while(uvIter!=edges.end()){
                int i=uvIter->first;
                int j=uvIter->second;
                L(i,j)=-1;
                L(j,i)=-1;
                L(i,i)++;
                L(j,j)++;
                Ls(i,j)=-1;
                Ls(j,i)=-1;
                Ls(i,i)++;
                Ls(j,j)++;
                uvIter++;
            }
            for(int i=0;i<fixedAnchorsNum;i++){
                Ls(pointsNum+i,fixedIndexes[i])=1;
            }
            for(int i=0;i<targetsNum;i++){
                Ls(pointsNum+fixedAnchorsNum+i,targetIndexes[i])=1;
            }

            A=Ls.transpose() * Ls;
            LDLT=A.ldlt();
            bx=L*vx;
            by=L*vy;
            bz=L*vz;
            bx.conservativeResize(pointsNum +fixedAnchorsNum+ targetsNum);
            by.conservativeResize(pointsNum +fixedAnchorsNum+ targetsNum);
            bz.conservativeResize(pointsNum +fixedAnchorsNum+ targetsNum);
            //std::cout<<"5555"<<std::endl;
            // 用形变前坐标对固定锚点坐标进行赋值
            for (auto i = 0; i < fixedAnchorsNum; i++)
            {
                bx(pointsNum+i) = vx(fixedIndexes[i]);
                by(pointsNum+i) = vy(fixedIndexes[i]);
                bz(pointsNum+i) = vz(fixedIndexes[i]);
            }
        }else{
            //std::cout<<"2222"<<std::endl;
            L.resize(roiNum,roiNum);
            Ls.resize(roiNum+fixedAnchorsNum+targetsNum,roiNum);
            VectorXT rvx=VectorXT::Zero(roiNum,1);
            VectorXT rvy=VectorXT::Zero(roiNum,1);
            VectorXT rvz=VectorXT::Zero(roiNum,1);
            for(int k=0;k<roi.size();k++){
                roiRowNum[roi[k]]=k;
                rvx[k]=vx(roi[k]);
                rvy[k]=vy(roi[k]);
                rvz[k]=vz(roi[k]);
            }
            //std::cout<<"3333"<<std::endl;
            std::set<std::pair<int,int>>::iterator uvIter = edges.begin();
            while(uvIter!=edges.end()){
                int i=uvIter->first;
                int j=uvIter->second;
                if(roiRowNum.count(i)&&roiRowNum.count(j)){
                    int iIndex=roiRowNum[i];
                    int jIndex=roiRowNum[j];
                    L(iIndex,jIndex)=-1;
                    L(jIndex,iIndex)=-1;
                    L(iIndex,iIndex)++;
                    L(jIndex,jIndex)++;
                    Ls(iIndex,jIndex)=-1;
                    Ls(jIndex,iIndex)=-1;
                    Ls(iIndex,iIndex)++;
                    Ls(jIndex,jIndex)++;
                }
                uvIter++;
            }
            //std::cout<<"4444"<<std::endl;
            for(int i=0;i<fixedAnchorsNum;i++){
                Ls(roiNum+i,roiRowNum[fixedIndexes[i]])=1;
            }
            for(int i=0;i<targetsNum;i++){
                Ls(roiNum+fixedAnchorsNum+i,roiRowNum[targetIndexes[i]])=1;
            }
            A=Ls.transpose() * Ls;
            LDLT=A.ldlt();
            bx=L*rvx;
            by=L*rvy;
            bz=L*rvz;
            bx.conservativeResize(roiNum +fixedAnchorsNum+ targetsNum);
            by.conservativeResize(roiNum +fixedAnchorsNum+ targetsNum);
            bz.conservativeResize(roiNum +fixedAnchorsNum+ targetsNum);
            //std::cout<<"5555"<<std::endl;
            // 用形变前坐标对固定锚点坐标进行赋值
            for (auto i = 0; i < fixedAnchorsNum; i++)
            {
                bx(roiNum+i) = vx(fixedIndexes[i]);
                by(roiNum+i) = vy(fixedIndexes[i]);
                bz(roiNum+i) = vz(fixedIndexes[i]);
            }
            //std::cout<<"6666"<<std::endl;
        }

    }
    Eigen::Matrix<Type, Eigen::Dynamic, 3,Eigen::RowMajor> dst;

    DenseLaplace(Type* data,int size,const std::vector<int>& triangles,const std::vector<int>& targetIndexes,std::vector<int> fixedIndexes=std::vector<int>())
        :targetIndexes(targetIndexes),fixedIndexes(fixedIndexes)
    {
        std::vector<Type> vertices(data,data+size);
        init(vertices,triangles,targetIndexes,fixedIndexes);
        //computeLaplaceMarix();
    }
    DenseLaplace(std::vector<Type>& vertices,const std::vector<int>& triangles,const std::vector<int>& targetIndexes,std::vector<int> fixedIndexes=std::vector<int>())
        :targetIndexes(targetIndexes),fixedIndexes(fixedIndexes){
        init(vertices,triangles,targetIndexes,fixedIndexes);
        //computeLaplaceMarix();
    }
    void updatePoints(const std::vector<Type> &vertices){
        assert(points.size()==vertices.size);
        memcpy(points.data(),vertices.data(),sizeof (Type)*vertices.size());
        updateCoordinate();
    }
    void updatePoints(const Type* data,int size){
        assert(points.size()==size);
        memcpy(points.data(),data,sizeof (Type)*size);
        updateCoordinate();
    }
    void solve(const Type* data,int size){
       std::vector<Type> targetVetices(data,data+size);
       solve(targetVetices);
    }
    void solve(const std::vector<Type>& targetVetices){
        assert(targetVetices.size()==3*targetsNum);
        if(roiNum==0){
            // 用形变后坐标对移动锚点坐标进行赋值
            for (auto i = 0; i < targetsNum; i++)
            {
                bx(pointsNum + fixedAnchorsNum+i) = targetVetices[3*i+0];
                by(pointsNum + fixedAnchorsNum+i) = targetVetices[3*i+1];
                bz(pointsNum + fixedAnchorsNum+i) = targetVetices[3*i+2];
            }

            VectorXT Lsbx=Ls.transpose()*bx;
            VectorXT Lsby=Ls.transpose()*by;
            VectorXT Lsbz=Ls.transpose()*bz;

            VectorXT dx=LDLT.solve(Lsbx);
            VectorXT dy=LDLT.solve(Lsby);
            VectorXT dz=LDLT.solve(Lsbz);

            dst.resize(pointsNum,3);
            dst.col(0)=dx;
            dst.col(1)=dy;
            dst.col(2)=dz;

            //update

            // 用形变前坐标对固定锚点坐标进行赋值
            for (auto i = 0; i < fixedAnchorsNum; i++)
            {
                int id=fixedIndexes[i];
                dst.row(id)=points.row(id);
            }
            // 用形变后坐标对移动锚点坐标进行赋值
            for (auto i = 0; i < targetsNum; i++)
            {
                int id=targetIndexes[i];
                dst(id,0)=targetVetices[3*i+0];
                dst(id,1)=targetVetices[3*i+1];
                dst(id,2)=targetVetices[3*i+2];
            }
        }else{
            // 用形变后坐标对移动锚点坐标进行赋值
            for (auto i = 0; i < targetsNum; i++)
            {
                bx(roiNum + fixedAnchorsNum+i) = targetVetices[3*i+0];
                by(roiNum + fixedAnchorsNum+i) = targetVetices[3*i+1];
                bz(roiNum + fixedAnchorsNum+i) = targetVetices[3*i+2];
            }

            VectorXT Lsbx=Ls.transpose()*bx;
            VectorXT Lsby=Ls.transpose()*by;
            VectorXT Lsbz=Ls.transpose()*bz;

            VectorXT dx=LDLT.solve(Lsbx);
            VectorXT dy=LDLT.solve(Lsby);
            VectorXT dz=LDLT.solve(Lsbz);

            dst=points;
            for(int k=0;k<roiNum;k++){
                int dstId=roi[k];
                dst(dstId,0)=dx(k);
                dst(dstId,1)=dy(k);
                dst(dstId,2)=dz(k);
            }

            //update

            // 用形变前坐标对固定锚点坐标进行赋值
            for (auto i = 0; i < fixedAnchorsNum; i++)
            {
                int id=fixedIndexes[i];
                dst.row(id)=points.row(id);
            }
            // 用形变后坐标对移动锚点坐标进行赋值
            for (auto i = 0; i < targetsNum; i++)
            {
                int id=targetIndexes[i];
                dst(id,0)=targetVetices[3*i+0];
                dst(id,1)=targetVetices[3*i+1];
                dst(id,2)=targetVetices[3*i+2];
            }
        }

    }
    void setRoi(std::vector<int> &value);
    Eigen::Matrix<Type, Eigen::Dynamic, 3, Eigen::RowMajor> getDst() const;
};

template<typename Type>
Eigen::Matrix<Type, Eigen::Dynamic, 3, Eigen::RowMajor> DenseLaplace<Type>::getDst() const
{
return dst;
}
template<typename Type>
void DenseLaplace<Type>::setRoi(std::vector<int> &value)
{
    roi=value;
//    roi=SetOperator<int>::setUnion(value,targetIndexes);
//    roi=SetOperator<int>::setUnion(roi,fixedIndexes);
    std::sort(roi.begin(), roi.end());
    //std::cout<<"1111111"<<std::endl;
    roiNum=roi.size();
    computeLaplaceMarix();
}
#endif // DENSELAPLACE_H




