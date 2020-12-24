#ifndef MESHTOOLS_H
#define MESHTOOLS_H
#include "uv_mapper/half_edge_mesh.hpp"
#include "uv_mapper/vec.hpp"
#include "../MeshDefinition.h"
class MeshTools
{
public:
    MeshTools();
    static std::vector<int> getBoundaries(std::vector<float>& inVertices,std::vector<int>& inFaces,int firstBoundId);
    static bool ReadMesh(Mesh & mesh, const std::string & filename);
    static bool ReadOBJ(Mesh & mesh, const std::string & filename);
    //static bool ReadOFF(Mesh & mesh, const std::string & filename);
    static bool WriteMesh(const Mesh & mesh, const std::string & filename, const std::streamsize & precision = 6);
    static bool WriteOBJ(const Mesh & mesh, const std::string & filename, const std::streamsize & precision = 6);
    static double Area(const Mesh & mesh);
    static double AverageEdgeLength(const Mesh & mesh);
    static bool HasBoundary(const Mesh & mesh);
    static bool HasOneComponent(const Mesh & mesh);
    static int Genus(const Mesh & mesh);
    static void BoundingBox(const Mesh & mesh, Mesh::Point & bmax, Mesh::Point & bmin);
    static void Reassign(const Mesh & mesh1, Mesh & mesh2);
};

#endif // MESHTOOLS_H
