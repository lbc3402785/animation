#ifndef POINTCLOUDREGISTER_H
#define POINTCLOUDREGISTER_H
//#include "TriMesh.h"
//#include "TriMesh_algo.h"
//#include "ICP.h"
#include "transform.h"
#include "util/eigenfunctions.h"
namespace Rigid{
template <typename Type>
class PointCloudRegister
{
public:
    PointCloudRegister(){}
    //static void registerPoints(Eigen::Matrix<Type, 3, Eigen::Dynamic>& srcPoints,Eigen::Matrix<Type, 3, Eigen::Dynamic>& dstPoints,double& error,int iter=3);
    //static void registerPoints(Eigen::Matrix<Type, 3, Eigen::Dynamic>& srcPoints,Eigen::Matrix<Type, 3, Eigen::Dynamic>& srcNormals,Eigen::Matrix<Type, 3, Eigen::Dynamic>& dstPoints,double& error,int iter=3);
    static void registerByCovariance(Eigen::Matrix<Type, 3, Eigen::Dynamic>& srcPoints,Eigen::Matrix<Type, 3, Eigen::Dynamic>& dstPoints);
    static void registerByCovariance(Eigen::Matrix<Type, 3, Eigen::Dynamic> &srcPoints,Eigen::Matrix<Type, 3, Eigen::Dynamic> &srcNormals,
                                                               Eigen::Matrix<Type, 3, Eigen::Dynamic> &dstPoints,Eigen::Matrix<Type, 3, Eigen::Dynamic> &dstNormals);
    static void globalRegister(Eigen::Matrix<Type, 3, Eigen::Dynamic>& srcPoints,Eigen::Matrix<Type, 3, Eigen::Dynamic>& dstPoints,Transform<Type>& transform);
    static void registerNPoint(Eigen::Matrix<Type, 3, Eigen::Dynamic> srcPoints,Eigen::Matrix<Type, 3, Eigen::Dynamic> dstPoints,Transform<Type>& Transform);
    //static bool trimmedICP(Eigen::Matrix<Type, 3, Eigen::Dynamic>& srcPoints,Eigen::Matrix<Type, 3, Eigen::Dynamic>& dstPoints,Transform<Type>& Transform,double& error,bool do_scale=false,bool do_affine=false,bool bulkmode=false);
};

//template <typename Type>
//bool PointCloudRegister<Type>::trimmedICP(Eigen::Matrix<Type, 3, Eigen::Dynamic>& srcPoints,Eigen::Matrix<Type, 3, Eigen::Dynamic>& dstPoints, Transform<Type>& transform,double& error,bool do_scale, bool do_affine, bool bulkmode)
//{
////    std::cout<<"srcPoints size:"<<srcPoints.rows()<<" "<<srcPoints.cols()<<std::endl;
////    std::cout<<"dstPoints size:"<<dstPoints.rows()<<" "<<dstPoints.cols()<<std::endl;
//    trimesh::TriMesh* mesh1=new trimesh::TriMesh();
//    trimesh::TriMesh *mesh2=new trimesh::TriMesh();
//    for(size_t i=0;i<srcPoints.cols();i++){
//        mesh1->vertices.push_back(trimesh::point(srcPoints(0,i),srcPoints(1,i),srcPoints(2,i)));
//        // if(srcPoints.normals(i)!=)
//        // mesh1->normals.push_back(trimesh::vec(srcPoints.normal(i).x(),srcPoints.normal(i).y(),srcPoints.normal(i).z()));
//    }
//    for(size_t i=0;i<dstPoints.cols();i++){
//        mesh2->vertices.push_back(trimesh::point(dstPoints(0,i),dstPoints(1,i),dstPoints(2,i)));
//        // if(dstPoints.normals(i)!=)
//        // mesh1->normals.push_back(trimesh::vec(dstPoints.normal(i).x(),dstPoints.normal(i).y(),dstPoints.normal(i).z()));
//    }
//    trimesh::xform xf1;
//    trimesh::xform xf2;
//    int verbose = 0;
//    trimesh::KDtree *kd1 = new  trimesh::KDtree(mesh1->vertices);
//    trimesh::KDtree *kd2 = new  trimesh::KDtree(mesh2->vertices);
//    std::vector<float> weights1, weights2;
//    //std::cout<<"kdtree init done"<<std::endl;
//    if (bulkmode) {
//        float area1 = mesh1->stat(trimesh::TriMesh::STAT_SUM, trimesh::TriMesh::STAT_FACEAREA);
//        float area2 = mesh2->stat(trimesh::TriMesh::STAT_SUM, trimesh::TriMesh::STAT_FACEAREA);
//        float overlap_area, overlap_dist;
//        trimesh::find_overlap(mesh1, mesh2, xf1, xf2, kd1, kd2,
//                              overlap_area, overlap_dist);
//        float frac_overlap = overlap_area / std::min(area1, area2);
//        if (frac_overlap < 0.1f) {
//            std::cerr<<"trimmed ICP Insufficient overlap"<<std::endl;
//            return false;
//        } else {
//            std::cout<<"trimmed ICP"<<frac_overlap<<" overlap"<<std::endl;
//        }
//    }
//    float err = trimesh::ICP(mesh1, mesh2, xf1, xf2, kd1, kd2, weights1, weights2,
//                             verbose, do_scale, do_affine);
//    if (err >= 0.0f){
//        error=err ;
//        std::cerr<<"ICP error:"<<err<<std::endl;
//    }else{
//        std::cerr<<"ICP failed!"<<std::endl;
//        return false;
//    }
//    Eigen::Matrix<Type, 3, 3> rotation;
//    Eigen::Matrix<Type, 3, 1> translation;
//    rotation(0,0)=*(xf2.begin()  )  ,  rotation(1,0)=*(xf2.begin()+1),  rotation(2,0)=*(xf2.begin()+2 ) ;
//    rotation(0,1)=*(xf2.begin()+4)  ,  rotation(1,1)=*(xf2.begin()+5),  rotation(2,1)=*(xf2.begin()+6 );
//    rotation(0,2)=*(xf2.begin()+8)  ,  rotation(1,2)=*(xf2.begin()+9),  rotation(2,2)=*(xf2.begin()+10);
//    translation(0)=*(xf2.begin()+12 ) ,  translation(1)=*(xf2.begin()+13),   translation(2)=*(xf2.begin()+14);
//    transform.setRotation(rotation);
//    transform.setTranslation(translation);
//    delete mesh1;
//    delete mesh2;
//    delete kd1;
//    delete kd2;
//    return true;
//}
//template<typename Type>
//void PointCloudRegister<Type>::registerPoints(Eigen::Matrix<Type, 3, Eigen::Dynamic>& srcPoints,Eigen::Matrix<Type, 3, Eigen::Dynamic>& srcNormals,Eigen::Matrix<Type, 3, Eigen::Dynamic>& dstPoints,double& error,int iter){
//    error=0;
//    double lastError=0;
//    Transform<Type> transform;
//    int cur=0;

//    while(cur<iter){
//       if(PointCloudRegister<Type>::trimmedICP(dstPoints,srcPoints,transform,error)){
//           std::cout<<"error:"<<error<<std::endl;
//           //std::cout<<transform<<std::endl;
//           if(cur>0&&error-lastError>0.05){
//               break;
//           }
//           transform.apply(srcPoints);
//           transform.applyRotation(srcNormals);
//           if(cur>0&std::fabs(error-lastError)<0.001){
//               break;
//           }

//           lastError=error;
//       }else{
//           break;
//       }
//       cur++;
//    }
//}

//template<typename Type>
//void PointCloudRegister<Type>::registerPoints(Eigen::Matrix<Type, 3, Eigen::Dynamic> &srcPoints, Eigen::Matrix<Type, 3, Eigen::Dynamic> &dstPoints,double& error, int iter)
//{
//    error=0;
//    double lastError=0;
//    Transform<Type> transform;
//    int cur=0;

//    while(cur<iter){
//       if(PointCloudRegister<Type>::trimmedICP(dstPoints,srcPoints,transform,error)){
//           std::cout<<"error:"<<error<<std::endl;
//           //std::cout<<transform<<std::endl;
//           if(cur>0&&error-lastError>0.05){
//               break;
//           }
//           transform.apply(srcPoints);
//           if(cur>0&std::fabs(error-lastError)<0.001){
//               break;
//           }

//           lastError=error;
//       }else{
//           break;
//       }
//       cur++;
//    }
//}
template<typename Type>
void PointCloudRegister<Type>::registerByCovariance(Eigen::Matrix<Type, 3, Eigen::Dynamic> &srcPoints,Eigen::Matrix<Type, 3, Eigen::Dynamic> &srcNormals,
                                                    Eigen::Matrix<Type, 3, Eigen::Dynamic> &dstPoints,Eigen::Matrix<Type, 3, Eigen::Dynamic> &dstNormals)
{
    Eigen::Matrix<Type, 3, 1> dstCenter=dstPoints.rowwise().mean();
    Eigen::Matrix<Type, 3, 1> srcCenter=srcPoints.rowwise().mean();
    Eigen::Matrix<Type, 3, 1> translation=dstCenter-srcCenter;
    std::cerr<<"translation:"<<translation<<std::endl<<std::flush;
    Transform<Type> t(translation);
    t.applyTranslation(srcPoints);
    Eigen::Matrix<Type, 3, 1> dstNormal=dstNormals.rowwise().mean().normalized();
    Eigen::Matrix<Type, 3, 1> srcNormal=srcNormals.rowwise().mean().normalized();
    std::cout<<"srcNormal:"<<std::endl<<srcNormal<<std::endl<<"dstNormal:"<<std::endl<<dstNormal<<std::endl<<std::flush;
    globalRegister(srcPoints,dstPoints,t);
    std::cerr<<"points rotation:"<<t.getRotation()<<std::endl<<std::flush;
    Transform<Type> t1;
    globalRegister(srcNormals,dstNormals,t1);
    std::cerr<<"normal rotation:"<<t1.getRotation()<<std::endl<<std::flush;
    t.applyRotation(srcPoints);
    t.applyRotation(srcNormals);
    srcNormal=srcNormals.rowwise().mean().normalized();
    std::cout<<"srcNormal:"<<std::endl<<srcNormal<<std::endl<<"dstNormal:"<<std::endl<<dstNormal<<std::endl<<std::flush;
    globalRegister(srcNormals,dstNormals,t1);
    std::cerr<<"normal rotation:"<<t1.getRotation()<<std::endl<<std::flush;
    t.applyTranslation(srcPoints);
}

template<typename Type>
void PointCloudRegister<Type>::registerByCovariance(Eigen::Matrix<Type, 3, Eigen::Dynamic> &srcPoints, Eigen::Matrix<Type, 3, Eigen::Dynamic> &dstPoints)
{
    Eigen::Matrix<Type, 3, 1> dstCenter=dstPoints.rowwise().mean();
    Eigen::Matrix<Type, 3, 1> srcCenter=srcPoints.rowwise().mean();
    Eigen::Matrix<Type, 3, 1> translation=dstCenter-srcCenter;
    std::cerr<<"translation:"<<translation<<std::endl<<std::flush;
    Transform<Type> t(translation);
    t.applyTranslation(srcPoints);
    globalRegister(srcPoints,dstPoints,t);
    t.apply(srcPoints);
}
template<typename Type>
void PointCloudRegister<Type>::globalRegister(Eigen::Matrix<Type, 3, Eigen::Dynamic> &srcPoints, Eigen::Matrix<Type, 3, Eigen::Dynamic> &dstPoints, Transform<Type> &t)
{
    Eigen::Matrix<Type, 3, 1> dstCenter=dstPoints.rowwise().mean();
    Eigen::Matrix<Type, 3, 3> A1=EigenFunctions<Type>::computeCovariance(dstPoints,dstCenter);
    Eigen::Matrix<Type, 3, 3> U1,S1,V1;
    EigenFunctions<Type>::svdDecomposition(A1,U1,S1,V1);

    Eigen::Matrix<Type, 3, 1> srcCenter=srcPoints.rowwise().mean();
    Eigen::Matrix<Type, 3, 3> A2=EigenFunctions<Type>::computeCovariance(srcPoints,srcCenter);
    Eigen::Matrix<Type, 3, 3> U2,S2,V2;
    EigenFunctions<Type>::svdDecomposition(A2,U2,S2,V2);

    Type c1=1;
    Type c2=1;
    Type c3=1;
//    std::cerr<<"A1:"<<std::endl<<A1<<std::endl<<"A2:"<<std::endl<<A2<<std::endl<<std::flush;
//    std::cerr<<"S1:"<<std::endl<<S1<<std::endl<<"S2:"<<std::endl<<S2<<std::endl<<std::flush;
//    std::cerr<<"U1:"<<std::endl<<U1<<std::endl<<"U2:"<<std::endl<<U2<<std::endl<<std::flush;
    if(U1.determinant()*U2.determinant()<0){
        Eigen::Matrix<Type, 3, 1> a1(U1(0,0),U1(1,0),U1(2,0));
        Eigen::Matrix<Type, 3, 1> b1(U2(0,0),U2(1,0),U2(2,0));
        Eigen::Matrix<Type, 3, 1> a2(U1(0,1),U1(1,1),U1(2,1));
        Eigen::Matrix<Type, 3, 1> b2(U2(0,1),U2(1,1),U2(2,1));
        Eigen::Matrix<Type, 3, 1> a3(U1(0,2),U1(1,2),U1(2,2));
        Eigen::Matrix<Type, 3, 1> b3(U2(0,2),U2(1,2),U2(2,2));
        c1=a1.transpose()*b1;
        c2=a2.transpose()*b2;
        c3=a3.transpose()*b3;
        std::cerr<<"c1:"<<c1<<" c2:"<<c2<<" c3:"<<c3<<std::endl<<std::flush;
        std::cerr<<"U1.determinant():"<<U1.determinant()<<std::endl<<" U2.determinant():"<<U2.determinant()<<std::endl<<std::flush;
        if(U1.determinant()<0&&U2.determinant()>0){
            if(c1*c2<0&&c2*c3>0){
                U1(0,0)*=-1,U1(1,0)*=-1,U1(2,0)*=-1;
            }else if(c1*c3>=0&&c2*c3<0){
                U1(0,1)*=-1,U1(1,1)*=-1,U1(2,1)*=-1;
            }else if(c1*c2>0&&c2*c3<0){
                U1(0,2)*=-1,U1(1,2)*=-1,U1(2,2)*=-1;
            }

        }else if(U1.determinant()>0&&U2.determinant()<0){
            if(c1*c2<0&&c2*c3>0){
                std::cerr<<"terrible:U1.determinant()>0&&U2.determinant()<0 c1*c2<0&&c2*c3>0 "<<std::endl<<std::flush;
                U2(0,0)*=-1,U2(1,0)*=-1,U2(2,0)*=-1;
            }else if(c1*c3>=0&&c2*c3<0){
                std::cerr<<"terrible:U1.determinant()>0&&U2.determinant()<0 c1*c3>=0&&c2*c3<0 "<<std::endl<<std::flush;
                U2(0,1)*=-1,U2(1,1)*=-1,U2(2,1)*=-1;
            }else if(c1*c2>0&&c2*c3<0){
                std::cerr<<"terrible:U1.determinant()>0&&U2.determinant()<0 c1*c2>0&&c2*c3<0 "<<std::endl<<std::flush;
                U2(0,2)*=-1,U2(1,2)*=-1,U2(2,2)*=-1;
            }
        }else if(U1.determinant()<0&&U2.determinant()<0){
            U1=-U1;
            U2=-U2;
            if(c1*c2<0&&c2*c3>0){
                std::cerr<<"terrible:U1.determinant()<0&&U2.determinant()<0 c1*c2<0&&c2*c3>0 "<<std::endl<<std::flush;
//                U1(0,0)*=-1,U1(1,0)*=-1,U1(2,0)*=-1;
                U1(0,1)*=-1,U1(1,1)*=-1,U1(2,1)*=-1;
                U1(0,2)*=-1,U1(1,2)*=-1,U1(2,2)*=-1;
            }else if(c1*c3>=0&&c2*c3<0){
//                U1(0,1)*=-1,U1(1,1)*=-1,U1(2,1)*=-1;
                U1(0,0)*=-1,U1(1,0)*=-1,U1(2,0)*=-1;
                U1(0,2)*=-1,U1(1,2)*=-1,U1(2,2)*=-1;
            }else if(c1*c2>0&&c2*c3<0){
//                U1(0,2)*=-1,U1(1,2)*=-1,U1(2,2)*=-1;
                U1(0,0)*=-1,U1(1,0)*=-1,U1(2,0)*=-1;
                U1(0,1)*=-1,U1(1,1)*=-1,U1(2,1)*=-1;
            }
        }else if(U1.determinant()>0&&U2.determinant()>0){
            if(c1*c2<0&&c2*c3>0){
                std::cerr<<"terrible:U1.determinant()>0&&U2.determinant()>0 c1*c2<0&&c2*c3>0 "<<std::endl<<std::flush;
//                U1(0,0)*=-1,U1(1,0)*=-1,U1(2,0)*=-1;
                U1(0,1)*=-1,U1(1,1)*=-1,U1(2,1)*=-1;
                U1(0,2)*=-1,U1(1,2)*=-1,U1(2,2)*=-1;
            }else if(c1*c3>=0&&c2*c3<0){
//                U1(0,1)*=-1,U1(1,1)*=-1,U1(2,1)*=-1;
                U1(0,0)*=-1,U1(1,0)*=-1,U1(2,0)*=-1;
                U1(0,2)*=-1,U1(1,2)*=-1,U1(2,2)*=-1;
            }else if(c1*c2>0&&c2*c3<0){
//                U1(0,2)*=-1,U1(1,2)*=-1,U1(2,2)*=-1;
                U1(0,0)*=-1,U1(1,0)*=-1,U1(2,0)*=-1;
                U1(0,1)*=-1,U1(1,1)*=-1,U1(2,1)*=-1;
            }
        }

//        std::cerr<<"after modify U1:"<<std::endl<<U1<<std::endl;
//        std::cerr<<"after modify U2:"<<std::endl<<U2<<std::endl;
//        std::cerr<<"U1.determinant():"<<U1.determinant()<<std::endl<<" U2.determinant():"<<U2.determinant()<<std::endl<<std::flush;
    }

    Eigen::Matrix<Type, 3, 3> R=U1*U2.transpose();
//    std::cerr<<"rotation:"<<R<<std::endl<<std::flush;
    Eigen::Matrix<Type, 3, 1> T=dstCenter-R*srcCenter;
    t.setRotation(R);
    t.setTranslation(T);
}
template<typename Type>
void PointCloudRegister<Type>::registerNPoint(Eigen::Matrix<Type, 3, Eigen::Dynamic> srcPoints,Eigen::Matrix<Type, 3, Eigen::Dynamic> dstPoints,Transform<Type>& transform){
    if(srcPoints.cols()!=dstPoints.cols()||srcPoints.cols()<3||dstPoints.cols()<3)
    {
        std::cerr<<"srcPoints.size():\t"<<srcPoints.size()<<std::endl;
        std::cerr<<"dstPoints.size():\t"<<dstPoints.size()<<std::endl;
        std::cerr<<"registrateNPoint points size donot match!"<<std::endl;

    }
    Eigen::Matrix<Type, 3, 1> srcCenter=srcPoints.rowwise().mean();
    Eigen::Matrix<Type, 3, 1> dstCenter=dstPoints.rowwise().mean();

    Eigen::Matrix<Type, 3, Eigen::Dynamic> srcMatrix=srcPoints.colwise()-srcCenter;
    Eigen::Matrix<Type, 3, Eigen::Dynamic> dstMatrix=dstPoints.colwise()-dstCenter;
    Eigen::Matrix<Type, 3, 3> A = srcMatrix * dstMatrix.transpose();

    Eigen::Matrix<Type, 3, 3> U, S, V;
    EigenFunctions<Type>::svdDecomposition(A,U,S,V);
    Eigen::Matrix<Type, 3, 3> matrixTemp = U * V;
    Type det = matrixTemp.determinant();

    Eigen::Matrix<Type, 3, 3> matrixM;
    matrixM<<1, 0, 0, 0, 1, 0, 0, 0, det;

    Eigen::Matrix<Type, 3, 3> R = V* matrixM * U.transpose();
    Eigen::Matrix<Type, 3, 1> T=dstCenter-R*srcCenter;

    transform.setRotation(R);
    transform.setTranslation(T);
}
}

#endif // POINTCLOUDREGISTER_H
