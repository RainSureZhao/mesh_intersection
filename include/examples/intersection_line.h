//
// Created by ByteDance on 9/4/24.
//

#ifndef INTERSECTION_LINE_H
#define INTERSECTION_LINE_H

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/intersection.h>
#include <CGAL/IO/PLY.h>
#include <vector>
#include <fstream>
#include <iostream>
#include <unordered_set>
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3 Point_3;
typedef K::Segment_3 Segment_3;
typedef K::Triangle_3 Triangle_3;
typedef CGAL::Surface_mesh<Point_3> Mesh;
namespace PMP = CGAL::Polygon_mesh_processing;

inline bool intersection_line(int argc, char* argv[]) {
    const std::string filename1 = (argc > 1) ? argv[1] : R"(D:\Code\Data\Plane.ply)";
    const std::string filename2 = (argc > 2) ? argv[2] : R"(D:\Code\Data\Plane2.ply)";
    const std::string output_filename = (argc > 3) ? argv[3] : R"(D:\Code\Data\intersection.ply)";
    // 读取第一个PLY文件
    Mesh mesh1;
    std::ifstream input1(filename1, std::ios::binary);
    if (!input1 || !CGAL::IO::read_PLY(input1, mesh1)) {
        std::cerr << "Cannot read file " << filename1 << std::endl;
        return false;
    }

    // 读取第二个PLY文件
    Mesh mesh2;
    std::ifstream input2(filename2, std::ios::binary);
    if (!input2 || !CGAL::IO::read_PLY(input2, mesh2)) {
        std::cerr << "Cannot read file " << filename2 << std::endl;
        return false;
    }

    std::unordered_set<Point_3> S;
    std::vector<Segment_3> intersection_segments;

    // 遍历第一个网格的所有面
    for (auto face1 : mesh1.faces()) {
        std::vector<Point_3> points1;
        for (auto v : CGAL::vertices_around_face(mesh1.halfedge(face1), mesh1)) {
            points1.push_back(mesh1.point(v));
        }
//        for(auto &p : points1) {
//            std::cout << p << "\n";
//        }
        Triangle_3 tri1(points1[0], points1[1], points1[2]);

        // 遍历第二个网格的所有面
        for (auto face2 : mesh2.faces()) {
            std::vector<Point_3> points2;
            for (auto v : CGAL::vertices_around_face(mesh2.halfedge(face2), mesh2)) {
                points2.push_back(mesh2.point(v));
            }
//            for(auto &p : points2) {
//                std::cout << p << "\n";
//            }
            Triangle_3 tri2(points2[0], points2[1], points2[2]);

            // 计算两个三角形的相交结果
            auto result = CGAL::intersection(tri1, tri2);
            if(result) {
                if (const Segment_3* s = boost::get<Segment_3>(&*result)) {
                    intersection_segments.push_back(*s);
                } else if (const Point_3* p = boost::get<Point_3>(&*result)) {
                    // 如果相交结果是一个点
                    intersection_segments.emplace_back(*p, *p); // 将点处理为长度为零的线段
                }
            }

        }
    }

    // 写入 PLY 文件
    std::ofstream output(output_filename, std::ios::binary);
    if (!output) {
        std::cerr << "Cannot open file " << output_filename << " for writing." << std::endl;
        return false;
    }

    for(auto &seg : intersection_segments) {
        S.insert(seg.source());
        S.insert(seg.target());
    }
    // 写入 PLY 头
    output << "ply\n";
    output << "format ascii 1.0\n";
    output << "element vertex " << S.size() << "\n";
    output << "property float x\n";
    output << "property float y\n";
    output << "property float z\n";
    output << "element edge " << intersection_segments.size() << "\n";
    output << "property int vertex1\n";
    output << "property int vertex2\n";
    output << "end_header\n";

    // 写入顶点
    std::map<Point_3, int> vertex_map;
    int vertex_index = 0;
    for (const auto& seg : intersection_segments) {
        if (vertex_map.find(seg.source()) == vertex_map.end()) {
            vertex_map[seg.source()] = vertex_index++;
            output << seg.source().x() << " " << seg.source().y() << " " << seg.source().z() << "\n";
        }
        if (vertex_map.find(seg.target()) == vertex_map.end()) {
            vertex_map[seg.target()] = vertex_index++;
            output << seg.target().x() << " " << seg.target().y() << " " << seg.target().z() << "\n";
        }
    }

    // 写入边（线段）
    for (const auto& seg : intersection_segments) {
        output << vertex_map[seg.source()] << " " << vertex_map[seg.target()] << "\n";
    }

    output.close();
    std::cout << "Intersection segments written to " << output_filename << std::endl;

    return true;
}

#endif //INTERSECTION_LINE_H
