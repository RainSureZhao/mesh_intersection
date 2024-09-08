#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/intersection.h>
#include <CGAL/IO/PLY.h>
#include <vector>
#include <fstream>
#include <iostream>
#include <unordered_set>
#include "examples/co_refinement.h"
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3 Point_3;
typedef K::Segment_3 Segment_3;
typedef K::Triangle_3 Triangle_3;
typedef CGAL::Surface_mesh<Point_3> Mesh;
namespace PMP = CGAL::Polygon_mesh_processing;

/**
 * input:
 * mesh1
 * point1: x, y, z
 * point2: x, y, z
 * point3: x, y, z
 *
 * mesh2
 * point1: x, y, z
 * point2: x, y, z
 * point3: x, y, z
 *
 * output:
 * mesh1:
 * point1: x, y, z
 * point2: x, y, z
 * point3: x, y, z
 *
 * mesh2:
 * point1: x, y, z
 * point2: x, y, z
 * point3: x, y, z
 *
 * boundary_points:
 * point1: x, y, z
 * point2: x, y, z
 * point3: x, y, z
 *
 * @param argc
 * @param argv
 * @return
 */



int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <input_file> <output_file>" << std::endl;
        return 1;
    }

    std::string input_file = argv[1];
    std::string output_file = argv[2];

    // Read input mesh data
    MeshData mesh1, mesh2;
    if (!ReadMeshData(input_file, mesh1, mesh2)) {
        return 1;
    }

    // Perform mesh clipping
    std::vector<Point3D> boundary_points = ClipMesh(mesh1.triangles, mesh2.triangles);

    // Write output mesh data
    WriteMeshData(output_file, mesh1, mesh2, boundary_points);

    return 0;
}
