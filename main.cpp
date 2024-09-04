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

int main(int argc, char* argv[]) {
    const std::string filename1 = (argc > 1) ? argv[1] : R"(../data/Cone.ply)";
    const std::string filename2 = (argc > 2) ? argv[2] : R"(../data/Plane.ply)";
    const std::string output_filename1 = (argc > 3) ? argv[3] : R"(../data/Cone_refine.ply)";
    const std::string output_filename2 = (argc > 4) ? argv[4] : R"(../data/Plane_refine.ply)";
    // 读取第一个PLY文件
    Mesh mesh1;
    std::ifstream input1(filename1, std::ios::binary);
    if (!input1 || !CGAL::IO::read_PLY(input1, mesh1)) {
        std::cerr << "Cannot read file " << filename1 << std::endl;
        return EXIT_FAILURE;
    }

    // 读取第二个PLY文件
    Mesh mesh2;
    std::ifstream input2(filename2, std::ios::binary);
    if (!input2 || !CGAL::IO::read_PLY(input2, mesh2)) {
        std::cerr << "Cannot read file " << filename2 << std::endl;
        return EXIT_FAILURE;
    }

    co_refinement_and_clip(mesh1, mesh2, output_filename1, output_filename2);

    return 0;
}
