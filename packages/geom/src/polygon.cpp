#include "geom/polygon.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/draw_triangulation_2.h>
#include <CGAL/mark_domain_in_triangulation.h>
#include <CGAL/Polygon_2.h>

#include <unordered_map>
#include <boost/property_map/property_map.hpp>

namespace truck::geom {

using CGAL_K = CGAL::Exact_predicates_inexact_constructions_kernel;
using CGAL_Vb = CGAL::Triangulation_vertex_base_2<CGAL_K>;
using CGAL_Fb = CGAL::Constrained_triangulation_face_base_2<CGAL_K>;
using CGAL_TDS = CGAL::Triangulation_data_structure_2<CGAL_Vb, CGAL_Fb>;
using CGAL_Itag = CGAL::Exact_predicates_tag;
using CGAL_CDT = CGAL::Constrained_Delaunay_triangulation_2<CGAL_K, CGAL_TDS, CGAL_Itag>;
using CGAL_Face_handle = CGAL_CDT::Face_handle;
using CGAL_Point = CGAL_CDT::Point;
using CGAL_Polygon = CGAL::Polygon_2<CGAL_K>;

std::vector<Triangle> Polygon::triangles() const noexcept {
    CGAL_CDT cgal_cdt;
    std::vector<Triangle> triangles;

    CGAL_Polygon cgal_polygon;
    for (const Vec2& point : *this) {
        cgal_polygon.push_back(CGAL_Point(point.x, point.y));
    }
    
    cgal_cdt.insert_constraint(
        cgal_polygon.vertices_begin(),
        cgal_polygon.vertices_end(),
        true
    );

    std::unordered_map<CGAL_Face_handle, bool> in_domain_map;
    boost::associative_property_map<std::unordered_map<CGAL_Face_handle, bool>> in_domain(in_domain_map);
    CGAL::mark_domain_in_triangulation(cgal_cdt, in_domain);

    for (const auto& cgal_face_it : cgal_cdt.finite_face_handles()) {
        if (get(in_domain, cgal_face_it)) {
            CGAL_Point p1 = cgal_face_it->vertex(0)->point();
            CGAL_Point p2 = cgal_face_it->vertex(1)->point();
            CGAL_Point p3 = cgal_face_it->vertex(2)->point();

            triangles.emplace_back(
                Triangle(
                    Vec2(p1.x(), p1.y()),
                    Vec2(p2.x(), p2.y()),
                    Vec2(p3.x(), p3.y())
                )
            );
        }
    }

    return triangles;
}

}  // namespace truck::geom