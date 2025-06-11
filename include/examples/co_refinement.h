//
// Created by RainSure on 24-9-3.
//

#ifndef MESH_INTERSECTION_CO_REFINEMENT_H
#define MESH_INTERSECTION_CO_REFINEMENT_H

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <iostream>
#include <array>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Surface_mesh<K::Point_3> Mesh;
typedef Mesh::Edge_index Edge_index;
typedef Mesh::Face_index Face_index;

namespace PMP = CGAL::Polygon_mesh_processing;

struct Point3D{
    Point3D() = default;
    Point3D(double x, double y, double z) : x(x), y(y), z(z) {}
    double x, y, z;
};

struct Triangle3D {
    Triangle3D() = default;
    Triangle3D(const std::array<Point3D, 3>& points) : points(points) {}
    Triangle3D(const Point3D& p1, const Point3D& p2, const Point3D& p3) : points({p1, p2, p3}) {}
    std::array<Point3D, 3> points;
};

struct MeshData {
    std::vector<Triangle3D> triangles;
};

// 二进制文件头结构
struct BinaryFileHeader {
    char signature[8];    // 文件标识 "MESH_BIN"
    int version;          // 版本号
    int mesh1_triangle_count;
    int mesh2_triangle_count;
    int boundary_point_count;
};

bool ReadMeshData(const std::string& file_path, MeshData& mesh1, MeshData& mesh2) {
    std::ifstream infile(file_path);
    if (!infile.is_open()) {
        std::cerr << "Error: Unable to open input file " << file_path << std::endl;
        return false;
    }

    // Read the mesh data
    std::string line;
    std::vector<Point3D> points;
    bool readingMesh1 = true;
    while (std::getline(infile, line)) {
        // 去掉前后的空格
        line.erase(0, line.find_first_not_of(" \t"));
        line.erase(line.find_last_not_of(" \t") + 1);

        if (line == "mesh1") {
            readingMesh1 = true;
        } else if (line == "mesh2") {
            readingMesh1 = false;
        } else if (!line.empty()) {
            // 直接读取 x, y, z
            double x, y, z;
            if (sscanf_s(line.c_str(), "%lf %lf %lf", &x, &y, &z) == 3) {
                points.emplace_back(x, y, z);
                if (points.size() == 3) {
                    Triangle3D triangle(points[0], points[1], points[2]);
                    if (readingMesh1) {
                        mesh1.triangles.push_back(triangle);
                    } else {
                        mesh2.triangles.push_back(triangle);
                    }
                    points.clear();
                }
            } else {
                std::cerr << "Error: Invalid format in line: " << line << std::endl;
                return false;
            }
        }
    }

    infile.close();
    return true;
}


void WriteMeshData(const std::string& file_path, const MeshData& mesh1, const MeshData& mesh2, const std::vector<Point3D>& boundary_points) {
    std::ofstream outfile(file_path);
    if (!outfile.is_open()) {
        std::cerr << "Error: Unable to open output file " << file_path << std::endl;
        return;
    }

    // Write mesh1
    outfile << "mesh1:\n";
    for (const auto& triangle : mesh1.triangles) {
        for (const auto& point : triangle.points) {
            outfile << point.x << " " << point.y << " " << point.z << "\n";
        }
    }

    // Write mesh2
    outfile << "mesh2:\n";
    for (const auto& triangle : mesh2.triangles) {
        for (const auto& point : triangle.points) {
            outfile << point.x << " " << point.y << " " << point.z << "\n";
        }
    }

    // Write boundary points
    outfile << "boundary_points:\n";
    for (const auto& point : boundary_points) {
        outfile << point.x << " " << point.y << " " << point.z << "\n";
    }

    outfile.close();
}

// 读取二进制文件
bool ReadMeshDataBinary(const std::string& file_path, MeshData& mesh1, MeshData& mesh2) {
    std::ifstream infile(file_path, std::ios::binary);
    if (!infile.is_open()) {
        std::cerr << "Error: Unable to open input file " << file_path << std::endl;
        return false;
    }

    // 读取文件头
    BinaryFileHeader header;
    infile.read(reinterpret_cast<char*>(&header), sizeof(header));

    // 验证文件签名
    if (std::memcmp(header.signature, "MESH_BIN", 8) != 0) {
        std::cerr << "Error: Invalid file format" << std::endl;
        return false;
    }

    // 读取 mesh1
    mesh1.triangles.resize(header.mesh1_triangle_count);
    infile.read(reinterpret_cast<char*>(mesh1.triangles.data()),
                sizeof(Triangle3D) * header.mesh1_triangle_count);

    // 读取 mesh2
    mesh2.triangles.resize(header.mesh2_triangle_count);
    infile.read(reinterpret_cast<char*>(mesh2.triangles.data()),
                sizeof(Triangle3D) * header.mesh2_triangle_count);

    infile.close();
    return true;
}

// 写入二进制文件
void WriteMeshDataBinary(const std::string& file_path, const MeshData& mesh1, const MeshData& mesh2,
                   const std::vector<Point3D>& boundary_points) {
    std::ofstream outfile(file_path, std::ios::binary);
    if (!outfile.is_open()) {
        std::cerr << "Error: Unable to open output file " << file_path << std::endl;
        return;
    }

    // 准备文件头
    BinaryFileHeader header;
    std::memcpy(header.signature, "MESH_BIN", 8);
    header.version = 1;
    header.mesh1_triangle_count = mesh1.triangles.size();
    header.mesh2_triangle_count = mesh2.triangles.size();
    header.boundary_point_count = boundary_points.size();

    // 写入文件头
    outfile.write(reinterpret_cast<const char*>(&header), sizeof(header));

    // 写入 mesh1
    outfile.write(reinterpret_cast<const char*>(mesh1.triangles.data()),
                  sizeof(Triangle3D) * mesh1.triangles.size());

    // 写入 mesh2
    outfile.write(reinterpret_cast<const char*>(mesh2.triangles.data()),
                  sizeof(Triangle3D) * mesh2.triangles.size());

    // 写入边界点
    outfile.write(reinterpret_cast<const char*>(boundary_points.data()),
                  sizeof(Point3D) * boundary_points.size());

    outfile.close();
}

inline void co_refinement(Mesh& mesh1, Mesh& mesh2, const std::string& output1, const std::string& output2) {
    // 检查输入网格是否是三角网格
    if (!CGAL::is_triangle_mesh(mesh1) || !CGAL::is_triangle_mesh(mesh2)) {
        std::cerr << "Error: Both meshes must be triangle meshes." << std::endl;
        return;
    }

    // 进行共精细化
    PMP::corefine(mesh1, mesh2);

    CGAL::IO::write_polygon_mesh(output1, mesh1, CGAL::parameters::stream_precision(17));
    CGAL::IO::write_polygon_mesh(output2, mesh2, CGAL::parameters::stream_precision(17));
    std::cout << "Co-refinement completed." << std::endl;
}

inline std::vector<K::Point_3> co_refinement_and_clip(Mesh& mesh1, Mesh& mesh2, const std::string& output1 = "", const std::string& output2 = "") {
    if(CGAL::Polygon_mesh_processing::does_self_intersect(mesh1)) {
        std::cerr << "Error: Mesh1 self-intersects." << std::endl;
        return {};
    }
    if(CGAL::Polygon_mesh_processing::does_self_intersect(mesh2)) {
        std::cerr << "Error: Mesh2 self-intersects." << std::endl;
        return {};
    }
    // 进行共精细化操作
    PMP::corefine(mesh1, mesh2);

    // 创建一个点在网格内外的检查器
    CGAL::Side_of_triangle_mesh<Mesh, K> inside(mesh1);

    // 标记需要删除的面
    std::vector<Face_index> faces_to_remove;

    std::vector<K::Point_3> points_in_boundary;

    for (Face_index f : mesh2.faces()) {
        // 获取三角形的三个顶点
        const auto h = mesh2.halfedge(f);
        const K::Point_3& p1 = mesh2.point(mesh2.target(h));
        const K::Point_3& p2 = mesh2.point(mesh2.target(mesh2.next(h)));
        const K::Point_3& p3 = mesh2.point(mesh2.target(mesh2.next(mesh2.next(h))));

        // 检查三角形的顶点是否都在mesh1的内部

        auto p1_inside_result = inside(p1);
        auto p2_inside_result = inside(p2);
        auto p3_inside_result = inside(p3);

        if ((p1_inside_result == CGAL::ON_BOUNDED_SIDE || p1_inside_result == CGAL::ON_BOUNDARY) &&
            (p2_inside_result == CGAL::ON_BOUNDED_SIDE || p2_inside_result == CGAL::ON_BOUNDARY) &&
            (p3_inside_result == CGAL::ON_BOUNDED_SIDE || p3_inside_result == CGAL::ON_BOUNDARY)) {
            faces_to_remove.push_back(f); // 如果全部点都在mesh1的内部或边界处，标记这个面
        }

        if (p1_inside_result == CGAL::ON_BOUNDARY) {
            points_in_boundary.push_back(p1);
        }
        if (p2_inside_result == CGAL::ON_BOUNDARY) {
            points_in_boundary.push_back(p2);
        }
        if (p3_inside_result == CGAL::ON_BOUNDARY) {
            points_in_boundary.push_back(p3);
        }
    }

    // 删除标记的面
    for (Face_index f : faces_to_remove) {
        CGAL::remove_face(f, mesh2);
    }

    if(!output1.empty()) CGAL::IO::write_polygon_mesh(output1, mesh1, CGAL::parameters::stream_precision(17));
    if(!output2.empty()) CGAL::IO::write_polygon_mesh(output2, mesh2, CGAL::parameters::stream_precision(17));
    std::cout << "Clipping completed, removed " << faces_to_remove.size() << " faces from mesh2." << std::endl;

    return points_in_boundary;
}

std::vector<Point3D> ClipMesh(std::vector<Triangle3D>& mesh1, std::vector<Triangle3D>& mesh2)
{
    // 将原始三角形构造成CGAL::Surface_Mesh
    Mesh cgal_mesh1, cgal_mesh2;
    // 将顶点去重后再加入到网格中
    std::map<K::Point_3, Mesh::Vertex_index> unique_vertices1, unique_vertices2;
    for (const auto& triangle : mesh1) {
        for (const auto& point : triangle.points) {
            if (unique_vertices1.find(K::Point_3(point.x, point.y, point.z)) == unique_vertices1.end()) {
                unique_vertices1[K::Point_3(point.x, point.y, point.z)] = cgal_mesh1.add_vertex(K::Point_3(point.x, point.y, point.z));
            }
        }
    }
    for (const auto& triangle : mesh2) {
        for (const auto& point : triangle.points) {
            if (unique_vertices2.find(K::Point_3(point.x, point.y, point.z)) == unique_vertices2.end()) {
                unique_vertices2[K::Point_3(point.x, point.y, point.z)] = cgal_mesh2.add_vertex(K::Point_3(point.x, point.y, point.z));
            }
        }
    }
    for (const auto& triangle : mesh1) {
//        Mesh::Vertex_index v0 = cgal_mesh1.add_vertex(K::Point_3(triangle.points[0].x, triangle.points[0].y, triangle.points[0].z));
//        Mesh::Vertex_index v1 = cgal_mesh1.add_vertex(K::Point_3(triangle.points[1].x, triangle.points[1].y, triangle.points[1].z));
//        Mesh::Vertex_index v2 = cgal_mesh1.add_vertex(K::Point_3(triangle.points[2].x, triangle.points[2].y, triangle.points[2].z));
        Mesh::Vertex_index v0 = unique_vertices1[K::Point_3(triangle.points[0].x, triangle.points[0].y, triangle.points[0].z)];
        Mesh::Vertex_index v1 = unique_vertices1[K::Point_3(triangle.points[1].x, triangle.points[1].y, triangle.points[1].z)];
        Mesh::Vertex_index v2 = unique_vertices1[K::Point_3(triangle.points[2].x, triangle.points[2].y, triangle.points[2].z)];
        cgal_mesh1.add_face(v0, v1, v2);
    }
    for (const auto& triangle : mesh2) {
//        Mesh::Vertex_index v0 = cgal_mesh2.add_vertex(K::Point_3(triangle.points[0].x, triangle.points[0].y, triangle.points[0].z));
//        Mesh::Vertex_index v1 = cgal_mesh2.add_vertex(K::Point_3(triangle.points[1].x, triangle.points[1].y, triangle.points[1].z));
//        Mesh::Vertex_index v2 = cgal_mesh2.add_vertex(K::Point_3(triangle.points[2].x, triangle.points[2].y, triangle.points[2].z));
        Mesh::Vertex_index v0 = unique_vertices2[K::Point_3(triangle.points[0].x, triangle.points[0].y, triangle.points[0].z)];
        Mesh::Vertex_index v1 = unique_vertices2[K::Point_3(triangle.points[1].x, triangle.points[1].y, triangle.points[1].z)];
        Mesh::Vertex_index v2 = unique_vertices2[K::Point_3(triangle.points[2].x, triangle.points[2].y, triangle.points[2].z)];
        cgal_mesh2.add_face(v0, v1, v2);
    }

    // 进行共精细化和裁剪操作
    auto boundary_points = co_refinement_and_clip(cgal_mesh1, cgal_mesh2);

    // 将裁剪后的网格转换为三角形列表
    std::vector<Point3D> result;
    for (const auto& point : boundary_points) {
        result.emplace_back(point.x(), point.y(), point.z());
    }
    // 修改mesh1和mesh2
    mesh1.clear();
    for (const auto& f : cgal_mesh1.faces()) {
        auto h = cgal_mesh1.halfedge(f);
        mesh1.emplace_back(
                Point3D(cgal_mesh1.point(cgal_mesh1.target(h)).x(), cgal_mesh1.point(cgal_mesh1.target(h)).y(), cgal_mesh1.point(cgal_mesh1.target(h)).z()),
                Point3D(cgal_mesh1.point(cgal_mesh1.target(cgal_mesh1.next(h))).x(), cgal_mesh1.point(cgal_mesh1.target(cgal_mesh1.next(h))).y(), cgal_mesh1.point(cgal_mesh1.target(cgal_mesh1.next(h))).z()),
                Point3D(cgal_mesh1.point(cgal_mesh1.target(cgal_mesh1.next(cgal_mesh1.next(h)))).x(), cgal_mesh1.point(cgal_mesh1.target(cgal_mesh1.next(cgal_mesh1.next(h)))).y(), cgal_mesh1.point(cgal_mesh1.target(cgal_mesh1.next(cgal_mesh1.next(h)))).z())
        );
    }
    mesh2.clear();
    for (const auto& f : cgal_mesh2.faces()) {
        auto h = cgal_mesh2.halfedge(f);
        mesh2.emplace_back(
                Point3D(cgal_mesh2.point(cgal_mesh2.target(h)).x(), cgal_mesh2.point(cgal_mesh2.target(h)).y(), cgal_mesh2.point(cgal_mesh2.target(h)).z()),
                Point3D(cgal_mesh2.point(cgal_mesh2.target(cgal_mesh2.next(h))).x(), cgal_mesh2.point(cgal_mesh2.target(cgal_mesh2.next(h))).y(), cgal_mesh2.point(cgal_mesh2.target(cgal_mesh2.next(h))).z()),
                Point3D(cgal_mesh2.point(cgal_mesh2.target(cgal_mesh2.next(cgal_mesh2.next(h)))).x(), cgal_mesh2.point(cgal_mesh2.target(cgal_mesh2.next(cgal_mesh2.next(h)))).y(), cgal_mesh2.point(cgal_mesh2.target(cgal_mesh2.next(cgal_mesh2.next(h)))).z())
        );
    }
    return result;
}

#endif //MESH_INTERSECTION_CO_REFINEMENT_H
