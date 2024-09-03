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

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Surface_mesh<K::Point_3> Mesh;
typedef Mesh::Edge_index Edge_index;
typedef Mesh::Face_index Face_index;

namespace PMP = CGAL::Polygon_mesh_processing;

void co_refinement(Mesh& mesh1, Mesh& mesh2) {
    // 检查输入网格是否是三角网格
    if (!CGAL::is_triangle_mesh(mesh1) || !CGAL::is_triangle_mesh(mesh2)) {
        std::cerr << "Error: Both meshes must be triangle meshes." << std::endl;
        return;
    }

    // 为mesh2 添加一个布尔属性，用于标记边是否是新增的
    auto is_new_edge_map = mesh2.add_property_map<Edge_index, bool>("e:is_new", false).first;
    // 进行共精细化
    PMP::corefine(mesh1, mesh2);

    std::cout << "Co-refinement completed." << std::endl;
}

void co_refinement_and_clip(Mesh& mesh1, Mesh& mesh2) {
    // 进行共精细化操作
    PMP::corefine(mesh1, mesh2);

    // 创建一个点在网格内外的检查器
    CGAL::Side_of_triangle_mesh<Mesh, K> inside(mesh1);

    // 标记需要删除的面
    std::vector<Face_index> faces_to_remove;

    for (Face_index f : mesh2.faces()) {
        // 获取三角形的三个顶点
        auto h = mesh2.halfedge(f);
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
    }

    // 删除标记的面
    for (Face_index f : faces_to_remove) {
        CGAL::remove_face(f, mesh2);
    }

    std::cout << "Clipping completed, removed " << faces_to_remove.size() << " faces from mesh2." << std::endl;
}

#endif //MESH_INTERSECTION_CO_REFINEMENT_H
