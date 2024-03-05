#pragma once

#include "fastgrid/grid.h"
#include "fastgrid/holder.h"

#include "geom/complex_polygon.h"
#include "geom/polygon.h"

namespace truck::fastgrid {

void Draw(const geom::Polygon& poly, U8Grid& grid);

void Draw(const geom::ComplexPolygon& complex_poly, U8Grid& grid);

}  // namespace truck::fastgrid