#include "geom/complex_polygon.h"

#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/draw_triangulation_2.h>
#include <CGAL/mark_domain_in_triangulation.h>

#include <boost/property_map/property_map.hpp>
#include <unordered_map>

namespace truck::geom {

using CgalK = CGAL::Exact_predicates_inexact_constructions_kernel;
using CgalVb = CGAL::Triangulation_vertex_base_2<CGAL_K>;
using CgalFb = CGAL::Constrained_triangulation_face_base_2<CGAL_K>;
using CgalTds = CGAL::Triangulation_data_structure_2<CGAL_Vb, CGAL_Fb>;
using CgalItag = CGAL::Exact_predicates_tag;
using CgalCdt = CGAL::Constrained_Delaunay_triangulation_2<CGAL_K, CGAL_TDS, CGAL_Itag>;
using CgalFaceHandle = CGAL_CDT::Face_handle;
using CgalPoint = CGAL_CDT::Point;
using CgalPolygon = CGAL::Polygon_2<CGAL_K>;

std::vector<Triangle> ComplexPolygon::triangles() const noexcept {
    CgalCdt cgal_cdt;
    std::vector<Triangle> triangles;

    CgalPolygon cgal_polygon_outer;
    for (const Vec2& point : outer) {
        cgal_polygon_outer.push_back(CGAL_Point(point.x, point.y));
    }

    cgal_cdt.insert_constraint(
        cgal_polygon_outer.vertices_begin(), cgal_polygon_outer.vertices_end(), true);

    for (const Polygon& inner : inners) {
        CgalPolygon cgal_polygon_inner;
        for (const Vec2& point : inner) {
            cgal_polygon_inner.push_back(CGAL_Point(point.x, point.y));
        }

        cgal_cdt.insert_constraint(
            cgal_polygon_inner.vertices_begin(), cgal_polygon_inner.vertices_end(), true);
    }

    std::unordered_map<CGAL_Face_handle, bool> in_domain_map;
    boost::associative_property_map<std::unordered_map<CGAL_Face_handle, bool>> const in_domain(
        in_domain_map);
    CGAL::mark_domain_in_triangulation(cgal_cdt, in_domain);

    for (const auto& cgal_face_it : cgal_cdt.finite_face_handles()) {
        if (get(in_domain, cgal_face_it)) {
            CgalPoint const p1 = cgal_face_it->vertex(0)->point();
            CgalPoint const p2 = cgal_face_it->vertex(1)->point();
            CgalPoint const p3 = cgal_face_it->vertex(2)->point();

            triangles.emplace_back(
                Vec2(p1.x(), p1.y()), Vec2(p2.x(), p2.y()), Vec2(p3.x(), p3.y()));
        }
    }

    return triangles;
}

Segments ComplexPolygon::segments() const noexcept {
    size_t size = outer.size();

    for (const auto& inner : inners) {
        size += inner.size();
    }

    Segments segments;
    segments.reserve(size);

    auto current_segments = outer.segments();
    segments.insert(segments.end(), current_segments.begin(), current_segments.end());

    for (const auto& inner : inners) {
        current_segments = inner.segments();
        segments.insert(segments.end(), current_segments.begin(), current_segments.end());
    }

    return segments;
}

}  // namespace truck::geom
