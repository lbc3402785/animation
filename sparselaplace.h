#ifndef SPARSELAPLACE_H
#define SPARSELAPLACE_H
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include<Eigen/SparseCholesky>
#include <vector>
#include <iostream>
#include <set>
#include <util/eigenfunctions.h>
template <typename Type>
class SparseLaplace
{
private:
    Eigen::Matrix<Type, Eigen::Dynamic, 3,Eigen::RowMajor> points;

    Eigen::Matrix<int, Eigen::Dynamic, 3,Eigen::RowMajor> faces;
    Eigen::SparseMatrix<Type> L;
    Eigen::SparseMatrix<Type> Ls;
    Eigen::SparseMatrix<Type> A;
    Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic> AD;
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<Type>> LDLT;
//    Eigen::LDLT<Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic>> LDLT_Dense;
//    Eigen::LLT<Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic>> LLT;
    Eigen::ConjugateGradient<Eigen::SparseMatrix<Type>, Eigen::Symmetric,Eigen::DiagonalPreconditioner< Type >> CG;
    typedef typename  Eigen::Matrix<Type,Eigen::Dynamic, 1> VectorXT;
    VectorXT bx;
    VectorXT by;
    VectorXT bz;
    int pointsNum;
    int fixedAnchorsNum;//may be zero
    int targetsNum;
    std::vector<int> targetIndexes;
    std::vector<int> fixedIndexes;
    std::pair<int,int> SortPair(int i,int j){
        if(i<j){
            return std::make_pair(i,j);
        }else{
            return std::make_pair(j,i);
        }
    }
    void init(std::vector<Type>& vertices,std::vector<int>& triangles,const std::vector<int>& targetIndexes,std::vector<int> fixedIndexes){
        assert(vertices.size()%3==0);
        assert(triangles.size()%3==0);
        points.resize(vertices.size()/3,3);
        faces.resize(triangles.size()/3,3);
        memcpy(points.data(),vertices.data(),sizeof (Type)*vertices.size());
        memcpy(faces.data(),triangles.data(),sizeof(int)*triangles.size());
        //EigenFunctions<Type>::saveEigenPoints(points.transpose(),faces.transpose(),"test.obj");
        pointsNum=points.rows();
        fixedAnchorsNum=fixedIndexes.size();
        targetsNum=targetIndexes.size();
        std::cout<<"pointsNum:"<<pointsNum<<std::endl;
        std::cout<<"fixedAnchorsNum:"<<fixedAnchorsNum<<std::endl;
        std::cout<<"targetsNum:"<<targetsNum<<std::endl;
        VectorXT vx=points.col(0);
        VectorXT vy=points.col(1);
        VectorXT vz=points.col(2);
        std::vector<Eigen::Triplet<float>> trips;
        std::vector<int> degrees(points.rows(),0);
        std::set<std::pair<int,int>> edges;
        for(int64_t i=0;i<faces.rows();i++){
            int i0=faces(i,0);
            int i1=faces(i,1);
            int i2=faces(i,2);
            edges.insert(SortPair(i0,i1));
            edges.insert(SortPair(i1,i2));
            edges.insert(SortPair(i2,i0));
        }
        //std::cout<<"11111"<<std::endl;
        std::set<std::pair<int,int>>::iterator uvIter = edges.begin();
        while(uvIter!=edges.end()){
            int i=uvIter->first;
            int j=uvIter->second;
            trips.emplace_back(Eigen::Triplet<float>(i,j,-1));
            trips.emplace_back(Eigen::Triplet<float>(j,i,-1));
            degrees[i]++;
            degrees[j]++;
            uvIter++;
        }

        for(int64_t i=0;i<pointsNum;i++){
            //std::cout<<"degrees["<<i<<"]:"<<degrees[i]<<std::endl;
            if(degrees[i]==0){
                std::cerr<<"points "<<i<<" is isolated point!"<<std::endl;
                exit(EXIT_FAILURE);
            }
            trips.emplace_back(Eigen::Triplet<float>(i,i,degrees[i]));
        }

        L.resize(pointsNum,pointsNum);
        L.setFromTriplets(trips.begin(),trips.end());//nxn
        Ls.resize(pointsNum+fixedAnchorsNum+targetsNum,pointsNum);
        for(int i=0;i<fixedAnchorsNum;i++){
            trips.emplace_back(Eigen::Triplet<float>(pointsNum+i,fixedIndexes[i],1));
        }
        for(int i=0;i<targetsNum;i++){
            trips.emplace_back(Eigen::Triplet<float>(pointsNum+fixedAnchorsNum+i,targetIndexes[i],1));
        }
        std::cout<<"2222"<<std::endl;

        Ls.setFromTriplets(trips.begin(),trips.end());//(n+fixAnchorsNum+68)xn
        std::cout<<"3333"<<std::endl;

        A=Ls.transpose() * Ls;
        Eigen::SparseMatrix<float> Id(A.rows(),A.rows());
        Id.setIdentity();
        A+=1e-5*Id;
        LDLT.compute(A);
        //        CG.compute(A);

        if(LDLT.info()!=Eigen::Success) {
            // decomposition failed
            std::cerr<<"decomposition failed"<<std::endl;
            exit(EXIT_FAILURE);
        }
        //        if(CG.info()!=Eigen::Success) {
        //          // decomposition failed
        //          std::cerr<<"decomposition failed"<<std::endl;
        //          exit(EXIT_FAILURE);
        //        }
        bx=L*vx;
        by=L*vy;
        bz=L*vz;
        bx.conservativeResize(pointsNum +fixedAnchorsNum+ targetsNum);
        by.conservativeResize(pointsNum +fixedAnchorsNum+ targetsNum);
        bz.conservativeResize(pointsNum +fixedAnchorsNum+ targetsNum);
        std::cout<<"5555"<<std::endl;
        // 用形变前坐标对固定锚点坐标进行赋值
        for (auto i = 0; i < fixedAnchorsNum; i++)
        {
            bx(pointsNum+i) = vx(fixedIndexes[i]);
            by(pointsNum+i) = vy(fixedIndexes[i]);
            bz(pointsNum+i) = vz(fixedIndexes[i]);
        }
    }
public:
    Eigen::Matrix<Type, Eigen::Dynamic, 3,Eigen::RowMajor> dst;
    SparseLaplace(Type* data,int size,std::vector<int>& triangles,const std::vector<int>& targetIndexes,std::vector<int> fixedIndexes=std::vector<int>())
        :targetIndexes(targetIndexes),fixedIndexes(fixedIndexes)
    {
        std::vector<Type> vertices(data,data+size);
        init(vertices,triangles,targetIndexes,fixedIndexes);
    }
    SparseLaplace(std::vector<Type>& vertices,std::vector<int>& triangles,const std::vector<int>& targetIndexes,std::vector<int> fixedIndexes=std::vector<int>())
        :targetIndexes(targetIndexes),fixedIndexes(fixedIndexes){
        init(vertices,triangles,targetIndexes,fixedIndexes);
    }
    void solve(const Type* data,int size){
        const std::vector<Type> targetVetices(data,data+size);
        solve(targetVetices);
    }

    void solve(const std::vector<Type>& targetVetices){
        assert(targetVetices.size()==3*targetsNum);
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
        if(LDLT.info()!=Eigen::Success) {
            // solving failed
            std::cerr<<"solving failed!"<<std::endl;
            return;
        }
        VectorXT dy=LDLT.solve(Lsby);
        if(LDLT.info()!=Eigen::Success) {
            // solving failed
            std::cerr<<"solving failed!"<<std::endl;
            return;
        }
        VectorXT dz=LDLT.solve(Lsbz);
        if(LDLT.info()!=Eigen::Success) {
            // solving failed
            std::cerr<<"solving failed!"<<std::endl;
            return;
        }

        //         VectorXT dz=CG.compute(A).solve(Lsbz);
        //         if(CG.info()!=Eigen::Success) {
        //           // solving failed
        //           std::cerr<<"solving failed!"<<std::endl;
        //           return;
        //         }

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
        //std::cout<<"===="<<std::endl;
    }
};
#endif // SPARSELAPLACE_H
