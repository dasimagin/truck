#include "navigation/viewer.h"

#include "common/exception.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace truck::navigation::viewer {

namespace {

cv::Scalar toCVScalar(const std::vector<int>& color_rgb) {
    VERIFY(color_rgb.size() == 3);
    return cv::Scalar(color_rgb[2], color_rgb[1], color_rgb[0]);
}

cv::Point toCVPoint(const geom::Vec2& origin, double res, const geom::Vec2& point) {
    return cv::Point(point.x * res, point.y * res) - cv::Point(origin.x, origin.y);
}

std::vector<cv::Point> toCVPoints(
    const geom::Vec2& origin, double res, const std::vector<geom::Vec2>& points) {
    std::vector<cv::Point> cv_points;

    for (const geom::Vec2& point : points) {
        cv_points.emplace_back(toCVPoint(origin, res, point));
    }

    return cv_points;
}


void drawPolygon(
    const ViewerParams& params, const geom::Vec2& origin, cv::Mat& frame,
    const geom::ComplexPolygon& polygon) {
    cv::fillPoly(
        frame,
        toCVPoints(origin, params.res, polygon.outer),
        toCVScalar(params.color_rgb.outer_polygon));

    for (const geom::Polygon& inner : polygon.inners) {
        cv::fillPoly(
            frame,
            toCVPoints(origin, params.res, inner),
            toCVScalar(params.color_rgb.inner_polygon));
    }
}

void drawSkeleton(
    const ViewerParams& params, const geom::Vec2& origin, cv::Mat& frame,
    const geom::Segments& skeleton) {
    for (const geom::Segment& seg : skeleton) {
        cv::line(
            frame,
            toCVPoint(origin, params.res, seg.begin),
            toCVPoint(origin, params.res, seg.end),
            toCVScalar(params.color_rgb.skeleton),
            params.thickness.skeleton);
    }
}

void drawLevelLines(
    const ViewerParams& params, const geom::Vec2& origin, cv::Mat& frame,
    const geom::Segments& level_lines) {
    for (const geom::Segment& seg : level_lines) {
        cv::line(
            frame,
            toCVPoint(origin, params.res, seg.begin),
            toCVPoint(origin, params.res, seg.end),
            toCVScalar(params.color_rgb.level_lines),
            params.thickness.level_lines);
    }
}

void drawMesh(
    const ViewerParams& params, const geom::Vec2& origin, cv::Mat& frame,
    const std::vector<geom::Vec2>& mesh) {
    for (const geom::Vec2& point : mesh) {
        cv::circle(
            frame,
            toCVPoint(origin, params.res, point),
            params.thickness.mesh,
            toCVScalar(params.color_rgb.mesh),
            cv::FILLED);
    }
}

void drawEdges(
    const ViewerParams& params, const geom::Vec2& origin, cv::Mat& frame,
    const std::vector<geom::Segment>& edges) {
    for (const geom::Segment& edge : edges) {
        cv::line(
            frame,
            toCVPoint(origin, params.res, edge.begin),
            toCVPoint(origin, params.res, edge.end),
            toCVScalar(params.color_rgb.edges),
            params.thickness.edges);
    }
}

}  // namespace

Viewer::Viewer() {}

void Viewer::draw(
    const ViewerParams& params, const geom::ComplexPolygons& polygons,
    const mesh::MeshBuild& mesh_build, const graph::GraphBuild& graph_build) {
    VERIFY(polygons.size() == 1);
    const auto& polygon = polygons[0];

    // set image borders via outer polygon's borders
    cv::Rect bb = cv::boundingRect(toCVPoints(geom::Vec2(0, 0), params.res, polygon.outer));
    cv::Mat frame = cv::Mat(bb.size(), CV_8UC3, toCVScalar(params.color_rgb.background));

    geom::Vec2 bb_origin(bb.x, bb.y);

    if (params.enable.polygon) {
        drawPolygon(params, bb_origin, frame, polygon);
    }

    if (params.enable.skeleton) {
        drawSkeleton(params, bb_origin, frame, mesh_build.skeleton);
    }

    if (params.enable.level_lines) {
        drawLevelLines(params, bb_origin, frame, mesh_build.level_lines);
    }

    if (params.enable.mesh) {
        drawMesh(params, bb_origin, frame, mesh_build.mesh);
    }

    if (params.enable.edges) {
        drawEdges(params, bb_origin, frame, graph_build.edges);
    }

    /**
     * Rotate an image around 'x' axis to avoid .png map mirroring
     * relative to the x-axis of the .geojson map
     */
    cv::flip(frame, frame, 0);
    cv::imwrite(params.path, frame);
}

}  // namespace truck::navigation::viewer