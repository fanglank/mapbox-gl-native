#pragma once

#include <array>
#include <limits>
#include <mbgl/util/geometry.hpp>

namespace mbgl {

// contains minX, minY, maxX, maxY
using WithinBBox = std::array<int64_t, 4>;
const WithinBBox DefaultBBox = WithinBBox{std::numeric_limits<int64_t>::max(),
                                          std::numeric_limits<int64_t>::max(),
                                          std::numeric_limits<int64_t>::min(),
                                          std::numeric_limits<int64_t>::min()};

// check if bbox1 is within bbox2
bool boxWithinBox(const WithinBBox& bbox1, const WithinBBox& bbox2);

void updateBBox(WithinBBox& bbox, const Point<int64_t>& p);

WithinBBox calculateBBox(const Geometry<int64_t>& geometries);

bool pointWithinPolygon(const Point<int64_t>& point, const Polygon<int64_t>& polygon, int64_t& shiftX, int64_t& shiftY);

bool pointWithinPolygons(const Point<int64_t>& point, const MultiPolygon<int64_t>& polygons, int64_t& shiftX, int64_t& shiftY);

bool lineStringWithinPolygon(const LineString<int64_t>& lineString, const Polygon<int64_t>& polygon, int64_t& shiftX, int64_t& shiftY);

bool lineStringWithinPolygons(const LineString<int64_t>& line, const MultiPolygon<int64_t>& polygons, int64_t& shiftX, int64_t& shiftY);

} // namespace mbgl

