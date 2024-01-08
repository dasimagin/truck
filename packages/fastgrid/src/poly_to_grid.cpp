#include "fastgrid/poly_to_grid.h"

#include "common/math.h"

namespace truck::fastgrid {

namespace impl {

__always_inline bool TryFitToGrid(
    geom::Vec2& rel_p1, geom::Vec2& rel_p2, const fastgrid::U8Grid& grid) noexcept {
    if (rel_p1.x > rel_p2.x) {
        std::swap(rel_p1, rel_p2);
    }
    const double max_x = grid.size.width * grid.resolution;
    if (rel_p2.x < 0 || max_x <= rel_p1.x) {
        return false;
    }
    double k = (rel_p2.y - rel_p1.y) / (rel_p2.x - rel_p1.x);
    if (rel_p1.x < 0) {
        rel_p1 = geom::Vec2(grid.resolution / 2, rel_p1.y + k * (grid.resolution / 2 - rel_p1.x));
    }
    if (max_x <= rel_p2.x) {
        rel_p2 = geom::Vec2(
            max_x - grid.resolution / 2, rel_p2.y + k * (max_x - grid.resolution / 2 - rel_p2.x));
    }
    if (rel_p1.y > rel_p2.y) {
        std::swap(rel_p1, rel_p2);
    }
    const double max_y = grid.size.height * grid.resolution;
    if (rel_p2.y < 0 || max_y <= rel_p1.y) {
        return false;
    }
    k = (rel_p2.x - rel_p1.x) / (rel_p2.y - rel_p1.y);
    if (rel_p1.y < 0) {
        rel_p1 = geom::Vec2(rel_p1.x + k * (grid.resolution / 2 - rel_p1.y), grid.resolution / 2);
    }
    if (max_y <= rel_p2.y) {
        rel_p2 = geom::Vec2(
            rel_p2.x + k * (max_y - grid.resolution / 2 - rel_p1.y), max_y - grid.resolution / 2);
    }
    return true;
}

__always_inline void DrivingByX(
    const geom::Vec2& rel_p1, const geom::Vec2& rel_p2, fastgrid::U8Grid& grid) noexcept {
    double k = (rel_p2.y - rel_p1.y) / (rel_p2.x - rel_p1.x);
    double y = rel_p1.y;
    for (int64_t x = static_cast<int64_t>(rel_p1.x / grid.resolution);
         x <= static_cast<int64_t>(rel_p2.x / grid.resolution);
         ++x, y += k) {
        grid.data[floor<int64_t, double>(y) * grid.size.width + x] = 1;
    }
}

__always_inline void DrivingByY(
    const geom::Vec2& rel_p1, const geom::Vec2& rel_p2, fastgrid::U8Grid& grid) noexcept {
    double k = (rel_p2.x - rel_p1.x) / (rel_p2.y - rel_p1.y);
    double x = rel_p1.x;
    for (int64_t y = static_cast<int64_t>(rel_p1.y / grid.resolution);
         y <= static_cast<int64_t>(rel_p2.y / grid.resolution);
         ++y, x += k) {
        grid.data[y * grid.size.width + floor<int64_t, double>(x)] = 1;
    }
}

__always_inline void SegmentToGrid(
    const geom::Vec2& p1, const geom::Vec2& p2, fastgrid::U8Grid& grid) noexcept {
    auto rel_p1 = grid.transform(p1);
    auto rel_p2 = grid.transform(p2);
    if (!TryFitToGrid(rel_p1, rel_p2, grid)) {
        return;
    }
    if (abs(rel_p2.x - rel_p1.x) >= abs(rel_p2.y - rel_p1.y)) {
        if (rel_p1.x <= rel_p2.x) {
            DrivingByX(rel_p1, rel_p2, grid);
        } else {
            DrivingByX(rel_p2, rel_p1, grid);
        }
    } else {
        if (rel_p1.y <= rel_p2.y) {
            DrivingByY(rel_p1, rel_p2, grid);
        } else {
            DrivingByY(rel_p2, rel_p1, grid);
        }
    }
}

__always_inline void PolyToGrid(const geom::Polygon& poly, fastgrid::U8Grid& grid) noexcept {
    for (auto it = poly.begin(); it + 1 != poly.end(); ++it) {
        SegmentToGrid(*it, *(it + 1), grid);
    }
    SegmentToGrid(*(poly.end() - 1), *poly.begin(), grid);
}

__always_inline void PolyToGrid(const geom::ComplexPolygon& poly, U8Grid& grid) noexcept {
    impl::PolyToGrid(poly.outer, grid);
    for (const auto& inner : poly.inners) {
        impl::PolyToGrid(inner, grid);
    }
}

}  // namespace impl

void PolyToGrid(const geom::Polygon& poly, U8Grid& grid) { impl::PolyToGrid(poly, grid); }

U8GridHolder PolyToGrid(
    const geom::Polygon& poly, const Size& size, double resolution,
    const std::optional<geom::Pose>& origin) {
    auto result = makeGrid<uint8_t>(size, resolution, origin);
    PolyToGrid(poly, *result);
    return result;
}

void PolyToGrid(const geom::ComplexPolygon& poly, U8Grid& grid) { impl::PolyToGrid(poly, grid); }

U8GridHolder PolyToGrid(
    const geom::ComplexPolygon& poly, const Size& size, double resolution,
    const std::optional<geom::Pose>& origin) {
    auto result = makeGrid<uint8_t>(size, resolution, origin);
    PolyToGrid(poly, *result);
    return result;
}

}  // namespace truck::fastgrid
