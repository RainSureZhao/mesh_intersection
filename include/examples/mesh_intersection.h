//
// Created by RainSure on 24-9-3.
//

#ifndef MESH_INTERSECTION_MESH_INTERSECTION_H
#define MESH_INTERSECTION_MESH_INTERSECTION_H


#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <iostream>
#include <string>
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Exact_predicates_exact_constructions_kernel EK;
typedef CGAL::Surface_mesh<K::Point_3> Mesh;
typedef boost::graph_traits<Mesh>::vertex_descriptor vertex_descriptor;
typedef Mesh::Property_map<vertex_descriptor,EK::Point_3> Exact_point_map;
namespace PMP = CGAL::Polygon_mesh_processing;
namespace params = CGAL::parameters;
struct Exact_vertex_point_map
{
    // typedef for the property map
    typedef boost::property_traits<Exact_point_map>::value_type value_type;
    typedef boost::property_traits<Exact_point_map>::reference reference;
    typedef boost::property_traits<Exact_point_map>::key_type key_type;
    typedef boost::read_write_property_map_tag category;
    // exterior references
    Exact_point_map exact_point_map;
    Mesh* tm_ptr;
    // Converters
    CGAL::Cartesian_converter<K, EK> to_exact;
    CGAL::Cartesian_converter<EK, K> to_input;
    Exact_vertex_point_map()
            : tm_ptr(nullptr)
    {}
    Exact_vertex_point_map(const Exact_point_map& ep, Mesh& tm)
            : exact_point_map(ep)
            , tm_ptr(&tm)
    {
        for (Mesh::Vertex_index v : vertices(tm))
            exact_point_map[v]=to_exact(tm.point(v));
    }
    friend
    reference get(const Exact_vertex_point_map& map, key_type k)
    {
        CGAL_precondition(map.tm_ptr!=nullptr);
        return map.exact_point_map[k];
    }
    friend
    void put(const Exact_vertex_point_map& map, key_type k, const EK::Point_3& p)
    {
        CGAL_precondition(map.tm_ptr!=nullptr);
        map.exact_point_map[k]=p;
        // create the input point from the exact one
        map.tm_ptr->point(k)=map.to_input(p);
    }
};

bool mesh_intersection(const std::string& file1, const std::string& file2)
{
    const std::string filename1 = (file1.size() > 0) ? file1 : CGAL::data_file_path(R"(D:\Code\Data\Sphere_10.ply)");
    const std::string filename2 = (file2.size() > 0) ? file2 : CGAL::data_file_path(R"(D:\Code\Data\Plane.ply)");
    Mesh mesh1, mesh2;

    if(!PMP::IO::read_polygon_mesh(filename1, mesh1) || !PMP::IO::read_polygon_mesh(filename2, mesh2))
    {
        std::cerr << "Invalid input." << std::endl;
        return false;
    }
    Exact_point_map mesh1_exact_points =
            mesh1.add_property_map<vertex_descriptor,EK::Point_3>("v:exact_point").first;
    Exact_point_map mesh2_exact_points =
            mesh2.add_property_map<vertex_descriptor,EK::Point_3>("v:exact_point").first;
    Exact_vertex_point_map mesh1_vpm(mesh1_exact_points, mesh1);
    Exact_vertex_point_map mesh2_vpm(mesh2_exact_points, mesh2);
    if ( PMP::corefine_and_compute_intersection(mesh1,
                                                mesh2,
                                                mesh1,
                                                params::vertex_point_map(mesh1_vpm),
                                                params::vertex_point_map(mesh2_vpm),
                                                params::vertex_point_map(mesh1_vpm) ) )
    {
        CGAL::IO::write_polygon_mesh("inter_intersection.off", mesh1, CGAL::parameters::stream_precision(17));
        if ( PMP::corefine_and_compute_union(mesh1,
                                             mesh2,
                                             mesh2,
                                             params::vertex_point_map(mesh1_vpm),
                                             params::vertex_point_map(mesh2_vpm),
                                             params::vertex_point_map(mesh2_vpm) ) )
        {
            std::cout << "Intersection and union were successfully computed\n";
            CGAL::IO::write_polygon_mesh("inter_union.off", mesh2, CGAL::parameters::stream_precision(17));
            return true;
        }
        std::cout << "Union could not be computed\n";
        return false;
    }
    std::cout << "Intersection could not be computed\n";
    return false;
}

#endif //MESH_INTERSECTION_MESH_INTERSECTION_H
