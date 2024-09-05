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
    for (const auto& triangle : mesh1) {
        Mesh::Vertex_index v0 = cgal_mesh1.add_vertex(K::Point_3(triangle.points[0].x, triangle.points[0].y, triangle.points[0].z));
        Mesh::Vertex_index v1 = cgal_mesh1.add_vertex(K::Point_3(triangle.points[1].x, triangle.points[1].y, triangle.points[1].z));
        Mesh::Vertex_index v2 = cgal_mesh1.add_vertex(K::Point_3(triangle.points[2].x, triangle.points[2].y, triangle.points[2].z));
        cgal_mesh1.add_face(v0, v1, v2);
    }
    for (const auto& triangle : mesh2) {
        Mesh::Vertex_index v0 = cgal_mesh2.add_vertex(K::Point_3(triangle.points[0].x, triangle.points[0].y, triangle.points[0].z));
        Mesh::Vertex_index v1 = cgal_mesh2.add_vertex(K::Point_3(triangle.points[1].x, triangle.points[1].y, triangle.points[1].z));
        Mesh::Vertex_index v2 = cgal_mesh2.add_vertex(K::Point_3(triangle.points[2].x, triangle.points[2].y, triangle.points[2].z));
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
