#include <mbgl/style/expression/within.hpp>

#include <mapbox/geojson.hpp>
#include <mapbox/geometry.hpp>
#include <mbgl/style/conversion/json.hpp>
#include <mbgl/tile/geometry_tile_data.hpp>

#include <mbgl/util/logging.hpp>
#include <mbgl/util/string.hpp>

#include <rapidjson/document.h>
#include <mbgl/math/clamp.hpp>

#include<algorithm>

namespace mbgl {
namespace {

Point<int64_t> latLonToTileCoodinates(const Point<double>& c, const mbgl::CanonicalTileID& canonical) {
    Point<int64_t> p;
    
    const double size = util::EXTENT * std::pow(2, canonical.z);
//    const double x0 = util::EXTENT * static_cast<double>(canonical.x);
//    const double y0 = util::EXTENT * static_cast<double>(canonical.y);
    
    auto x = (c.x + 180.0) * size / 360.0;
//    auto x = (c.x + 180.0) * size / 360.0 - x0;
    p.x =(util::clamp<int64_t>(x, std::numeric_limits<int64_t>::min(), std::numeric_limits<int64_t>::max()));

    auto y = (180 - (std::log(std::tan((c.y + 90) * M_PI / 360.0)) * 180 / M_PI)) * size / 360;
//    auto y = (180 - (std::log(std::tan((c.y + 90) * M_PI / 360.0)) * 180 / M_PI)) * size / 360 - y0;
    p.y =(util::clamp<int64_t>(y, std::numeric_limits<int64_t>::min(), std::numeric_limits<int64_t>::max()));

    return p;
};

Polygon<int64_t> getTilePolygon(const Polygon<double>& polygon, const mbgl::CanonicalTileID& canonical, WithinBBox& bbox, int64_t& shiftX, int64_t& shiftY) {
     Polygon<int64_t> result;
     result.reserve(polygon.size());
     for (const auto ring : polygon) {
         LinearRing<int64_t> temp;
         temp.reserve(ring.size());
         for (const auto p : ring) {
             const auto coord = latLonToTileCoodinates(p, canonical);
             if (coord.x < 0) shiftX = std::max(shiftX, std::abs(coord.x));
             if (coord.y < 0) shiftX = std::max(shiftY, std::abs(coord.y));
             temp.emplace_back(coord);
             updateBBox(bbox, coord);
         }
         result.emplace_back(temp);
     }
     return result;
}

MultiPolygon<int64_t> getTilePolygons(const MultiPolygon<double> polygons, const mbgl::CanonicalTileID& canonical, WithinBBox& bbox, int64_t& shiftX, int64_t& shiftY) {
      MultiPolygon<int64_t> result;
      result.reserve(polygons.size());
      for (const auto pg : polygons) {
          result.push_back(getTilePolygon(pg, canonical, bbox, shiftX, shiftY));
      }
      return result;
}


bool pointsWithinPolygons(const mbgl::GeometryTileFeature& feature,
                          const mbgl::CanonicalTileID& canonical,
                          const Feature::geometry_type& polygonGeoSet) {
    
    return polygonGeoSet.match(
        [&feature, &canonical](const mapbox::geometry::multi_polygon<double>& polys) -> bool {
            WithinBBox polyBBox= DefaultBBox;
            int64_t shiftX = 0, shiftY = 0;
            auto polygons = getTilePolygons(polys, canonical, polyBBox, shiftX, shiftY);
            
            const GeometryCollection& geometries = feature.getGeometries();
            const int64_t x0 = util::EXTENT * canonical.x;
            const int64_t y0 = util::EXTENT * canonical.y;
            switch (feature.getType()) {
                case FeatureType::Point: {
                    MultiPoint<int64_t> points;
                    WithinBBox pointBBox = DefaultBBox;
                    for (const auto& p : geometries.at(0)) {
                        auto point = Point<int64_t>(p.x + x0, p.y + y0);
                        updateBBox(pointBBox, point);
                        points.push_back(point);
//                        if (!pointWithinPolygons({p.x, p.y}, polygons, shiftX, shiftY)) return false;
                    }
                    if (!boxWithinBox(pointBBox, polyBBox)) return false;
            
                    return std::all_of(points.begin(), points.end(), [&polygons, &shiftX, &shiftY](const auto& p) {
                        return pointWithinPolygons(p, polygons, shiftX, shiftY);
                                       });
            
                }
                case FeatureType::LineString: {
                    MultiLineString<int64_t> multiLineString;
                    WithinBBox lineBBox = DefaultBBox;
                    for (const auto& line : geometries) {
                        LineString<int64_t> lineString;
                        for (const auto& p : line) {
                            auto point = Point<int64_t>(p.x + x0, p.y + y0);
                            updateBBox(lineBBox, point);
                            lineString.emplace_back(point);
                        }
                        multiLineString.push_back(std::move(lineString));
                    }
                    if (!boxWithinBox(lineBBox, polyBBox)) return false;
                    if (multiLineString.size() == 1) {
                        return lineStringWithinPolygons(multiLineString[0], polygons, shiftX, shiftY);
                    }
                    return std::all_of(multiLineString.begin(), multiLineString.end(), [&polygons, &shiftX, &shiftY](const auto& line) {
                                          return lineStringWithinPolygons(line, polygons, shiftX, shiftY);
                                      });
                }
                default: return false;
            };
        },
        [&feature, &canonical](const mapbox::geometry::polygon<double>& poly) -> bool {
            WithinBBox polyBBox= DefaultBBox;
            int64_t shiftX = 0, shiftY = 0;
            auto polygon = getTilePolygon(poly, canonical, polyBBox, shiftX, shiftY);
            
           const GeometryCollection& geometries = feature.getGeometries();
            const int64_t x0 = util::EXTENT * canonical.x;
            const int64_t y0 = util::EXTENT *canonical.y;
            switch (feature.getType()) {
                case FeatureType::Point: {
                    WithinBBox pointBBox = DefaultBBox;
                    MultiPoint<int64_t> points;
                    for (const auto& p : geometries.at(0)) {
                        auto point = Point<int64_t>(p.x + x0, p.y + y0);
                        updateBBox(pointBBox, point);
                        points.push_back(std::move(point));
                    }
                    if (!boxWithinBox(pointBBox, polyBBox)) return false;
                    
                    return std::all_of(points.begin(), points.end(), [&polygon, &shiftX, &shiftY](const auto& p) {
                           return pointWithinPolygon(p, polygon, shiftX, shiftY);
                                          });
            
                }
                case FeatureType::LineString: {
                    MultiLineString<int64_t> multiLineString;
                    WithinBBox lineBBox = DefaultBBox;
                    for (const auto& line : geometries) {
                        LineString<int64_t> lineString;
                        for (const auto& p : line) {
                            auto point = Point<int64_t>(p.x + x0, p.y + y0);
                            updateBBox(lineBBox, point);
                            lineString.push_back(std::move(point));
                        }
                        multiLineString.push_back(std::move(lineString));
                    }
                    if(!boxWithinBox(lineBBox, polyBBox)) return false;
                    if (multiLineString.size() == 1) {
                        return lineStringWithinPolygon(multiLineString[0], polygon, shiftX, shiftY);
                    }
                      return std::all_of(multiLineString.begin(), multiLineString.end(), [&polygon, &shiftX, &shiftY](const auto& line) {
                                          return lineStringWithinPolygon(line, polygon, shiftX, shiftY);
                                      });
                }
                default: return false;
            };
        },
        [](const auto&) -> bool { return false; });
}

mbgl::optional<mbgl::GeoJSON> parseValue(const mbgl::style::conversion::Convertible& value_,
                                         mbgl::style::expression::ParsingContext& ctx) {
    if (isObject(value_)) {
        mbgl::style::conversion::Error error;
        auto geojson = toGeoJSON(value_, error);
        if (geojson && error.message.empty()) {
            return geojson;
        }
        ctx.error(error.message);
    }

    ctx.error("'within' expression requires valid geojson source that contains polygon geometry type.");
    return nullopt;
}

struct PolygonInfo {
    PolygonInfo(const Feature::geometry_type& geometry_) : geometry(geometry_){};
    Feature::geometry_type geometry;
};

mbgl::optional<PolygonInfo> getPolygonInfo(const Feature& polyFeature, mbgl::style::expression::ParsingContext& ctx) {
    const auto type = apply_visitor(ToFeatureType(), polyFeature.geometry);
    if (type == FeatureType::Polygon) {
        return PolygonInfo(polyFeature.geometry);
    }
    ctx.error("'within' expression requires valid geojson source that contains polygon geometry type.");
    return nullopt;
}
} // namespace

namespace style {
namespace expression {

Within::Within(GeoJSON geojson, Feature::geometry_type geometries_)
    : Expression(Kind::Within, type::Boolean),
      geoJSONSource(std::move(geojson)),
      geometries(std::move(geometries_)) {}

Within::~Within() = default;

using namespace mbgl::style::conversion;

EvaluationResult Within::evaluate(const EvaluationContext& params) const {
    if (!params.feature || !params.canonical) {
        return false;
    }
    auto geometryType = params.feature->getType();
    // Currently only support Point/Points in Polygon/Polygons
    if (geometryType == FeatureType::Point) {
        return pointsWithinPolygons(*params.feature, *params.canonical, geometries);
    } else if (geometryType == FeatureType::LineString) {
        return pointsWithinPolygons(*params.feature, *params.canonical, geometries);
    }
    mbgl::Log::Warning(mbgl::Event::General,
                       "within expression currently only support Point/LineString geometry type.");

    return false;
}

ParseResult Within::parse(const Convertible& value, ParsingContext& ctx) {
    if (isArray(value)) {
        // object value, quoted with ["within", value]
        if (arrayLength(value) != 2) {
            ctx.error("'within' expression requires exactly one argument, but found " +
                      util::toString(arrayLength(value) - 1) + " instead.");
            return ParseResult();
        }

        auto parsedValue = parseValue(arrayMember(value, 1), ctx);
        if (!parsedValue) {
            return ParseResult();
        }

        return parsedValue->match(
            [&parsedValue, &ctx](const mapbox::geometry::geometry<double>& geometrySet) {
                if (auto ret = getPolygonInfo(mbgl::Feature(geometrySet), ctx)) {
                    return ParseResult(std::make_unique<Within>(*parsedValue, std::move(ret->geometry)));
                }
                return ParseResult();
            },
            [&parsedValue, &ctx](const mapbox::feature::feature<double>& feature) {
                if (auto ret = getPolygonInfo(mbgl::Feature(feature), ctx)) {
                    return ParseResult(std::make_unique<Within>(*parsedValue, std::move(ret->geometry)));
                }
                return ParseResult();
            },
            [&parsedValue, &ctx](const mapbox::feature::feature_collection<double>& features) {
                for (const auto& feature : features) {
                    if (auto ret = getPolygonInfo(mbgl::Feature(feature), ctx)) {
                        return ParseResult(std::make_unique<Within>(*parsedValue, std::move(ret->geometry)));
                    }
                }
                return ParseResult();
            },
            [&ctx](const auto&) {
                ctx.error("'within' expression requires valid geojson source that contains polygon geometry type.");
                return ParseResult();
            });
    }
    ctx.error("'within' expression needs to be an array with exactly one argument.");
    return ParseResult();
}

Value valueConverter(const mapbox::geojson::rapidjson_value& v) {
    if (v.IsDouble()) {
        return v.GetDouble();
    }
    if (v.IsString()) {
        return std::string(v.GetString());
    }
    if (v.IsArray()) {
        std::vector<Value> result;
        result.reserve(v.Size());
        for (const auto& m : v.GetArray()) {
            result.push_back(valueConverter(m));
        }
        return result;
    }
    if (v.IsObject()) {
        std::unordered_map<std::string, Value> result;
        for (const auto& m : v.GetObject()) {
            result.emplace(m.name.GetString(), valueConverter(m.value));
        }
        return result;
    }
    // Ignore other types as valid geojson only contains above types.
    return Null;
}

mbgl::Value Within::serialize() const {
    std::unordered_map<std::string, Value> serialized;
    rapidjson::CrtAllocator allocator;
    const mapbox::geojson::rapidjson_value value = mapbox::geojson::convert(geoJSONSource, allocator);
    if (value.IsObject()) {
        for (const auto& m : value.GetObject()) {
            serialized.emplace(m.name.GetString(), valueConverter(m.value));
        }
    } else {
        mbgl::Log::Error(mbgl::Event::General,
                         "Failed to serialize 'within' expression, converted rapidJSON is not an object");
    }
    return std::vector<mbgl::Value>{{getOperator(), *fromExpressionValue<mbgl::Value>(serialized)}};
}

bool Within::operator==(const Expression& e) const {
    if (e.getKind() == Kind::Within) {
        auto rhs = static_cast<const Within*>(&e);
        return geoJSONSource == rhs->geoJSONSource && geometries == rhs->geometries;
    }
    return false;
}

std::vector<optional<Value>> Within::possibleOutputs() const {
    return {{true}, {false}};
}

std::string Within::getOperator() const {
    return "within";
}

} // namespace expression
} // namespace style
} // namespace mbgl
