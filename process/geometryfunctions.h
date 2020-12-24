#ifndef GEOMETRYFUNCTIONS_H
#define GEOMETRYFUNCTIONS_H


#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/Search_traits_3.h>
#include <CGAL/Search_traits_adapter.h>
#include <CGAL/point_generators_3.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/property_map.h>
#include <boost/iterator/zip_iterator.hpp>
#include <utility>
template <typename Kernel,typename Type>
class GeometryFunctions{
public:
    static Type calSingleDistance(Eigen::Matrix<Type, 3, 1>& p,Eigen::Matrix<Type, 3, Eigen::Dynamic>& modelPoints,
                                  const Eigen::Matrix<int, 3, Eigen::Dynamic>& faces);
    static Type calDistance(Eigen::Matrix<Type, 3, Eigen::Dynamic>& cloudPoints, Eigen::Matrix<Type, 3, Eigen::Dynamic>& modelPoints,
                            Eigen::Matrix<int, 3, Eigen::Dynamic>& faces,Eigen::Matrix<Type, Eigen::Dynamic, 1>& curvatures,Type threshold,
                            Eigen::Matrix<Type, Eigen::Dynamic,1>& result);
};

template<typename Kernel, typename Type>
Type GeometryFunctions<Kernel,Type>::calSingleDistance(Eigen::Matrix<Type, 3, 1> &p, Eigen::Matrix<Type, 3, Eigen::Dynamic> &modelPoints,const Eigen::Matrix<int, 3, Eigen::Dynamic> &faces)
{
    typedef typename Kernel::FT FT;
    typedef typename Kernel::Point_3 Point;
    typedef typename Kernel::Triangle_3 Triangle;//Triangle
    typedef typename Kernel::Segment_3 Segment;
    typedef typename std::vector<Triangle>::iterator Iterator;
    typedef typename CGAL::AABB_triangle_primitive<Kernel, Iterator> Primitive;
    typedef typename CGAL::AABB_traits<Kernel, Primitive> Traits;
    typedef typename CGAL::AABB_tree<Traits> Tree;
    typedef typename Tree::Point_and_primitive_id Point_and_primitive_id;
    std::vector<Triangle> triangles;//mesh.faces

    for (int i = 0; i < faces.cols(); i++)
    {
        int j,k,l;
        j=faces(0,i)-1;
        k=faces(1,i)-1;
        l=faces(2,i)-1;
        Point p1(modelPoints(0,j), modelPoints(1,j),modelPoints(2,j));
        Point p2(modelPoints(0,k), modelPoints(1,k),modelPoints(2,k));
        Point p3(modelPoints(0,l), modelPoints(1,l),modelPoints(2,l));
        triangles.push_back(Triangle(p1, p2, p3));
    }

    Tree tree(triangles.begin(), triangles.end());
    tree.accelerate_distance_queries();
    Point query(p(0,0),p(1,0),p(2,0));
    FT sqd = tree.squared_distance(query);
    Type cur=std::sqrt(sqd);
    return cur;
}

/**
 * 作者：Annora-W
 * 来源：CSDN
 * 原文：https://blog.csdn.net/wwanrong/article/details/78831580
 */
template<typename Kernel, typename Type>
Type GeometryFunctions<Kernel,Type>::calDistance(Eigen::Matrix<Type, 3, Eigen::Dynamic>& cloudPoints, Eigen::Matrix<Type, 3, Eigen::Dynamic>& modelPoints,Eigen::Matrix<int, 3, Eigen::Dynamic>& faces,
                                                 Eigen::Matrix<Type,Eigen::Dynamic, 1>& curvatures,Type threshold, Eigen::Matrix<Type, Eigen::Dynamic,1>& result)
{
    typedef typename Kernel::FT FT;
    typedef typename Kernel::Point_3 Point;
    typedef typename Kernel::Triangle_3 Triangle;//Triangle
    typedef typename Kernel::Segment_3 Segment;
    typedef typename std::vector<Triangle>::iterator Iterator;
    typedef typename CGAL::AABB_triangle_primitive<Kernel, Iterator> Primitive;
    typedef typename CGAL::AABB_traits<Kernel, Primitive> Traits;
    typedef typename CGAL::AABB_tree<Traits> Tree;
    typedef typename Tree::Point_and_primitive_id Point_and_primitive_id;
    std::vector<Triangle> triangles;//mesh.faces

    for (int i = 0; i < faces.cols(); i++)
    {
        int j,k,l;
        j=faces(0,i)-1;
        k=faces(1,i)-1;
        l=faces(2,i)-1;
        Point p1(modelPoints(0,j), modelPoints(1,j),modelPoints(2,j));
        Point p2(modelPoints(0,k), modelPoints(1,k),modelPoints(2,k));
        Point p3(modelPoints(0,l), modelPoints(1,l),modelPoints(2,l));
        triangles.push_back(Triangle(p1, p2, p3));
    }

    Tree tree(triangles.begin(), triangles.end());
    tree.accelerate_distance_queries();
    // computes squared distance from query
    float sum=0.0;
    float weightSum=0.0;
    for(int i=0;i<cloudPoints.cols();i++){
        Point query(cloudPoints(0,i),cloudPoints(1,i),cloudPoints(2,i));
        FT sqd = tree.squared_distance(query);
        Type cur=std::sqrt(sqd);
//        if(cur>threshold){
//            cur=threshold;
//        }else{
            sum+=cur*curvatures(i);
            //if(cur>5)std::cout  <<"cur curvature:"<<curvatures(index) << " cur: "<<cur<< std::endl;
            weightSum+=curvatures(i);
        //}

        result(i)=cur;
    }

    return sum/weightSum;
}
#endif // GEOMETRYFUNCTIONS_H
