#include "Utils.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <limits>
std::vector<Point> loadPoints(const std::string& filename) {
    std::vector<Point> points;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return points;
    }
    double x, y;
    while (file >> x >> y) {
        points.emplace_back(x, y);
    }
    return points;
}
void saveEdges(const std::vector<Edge>& edges, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file for writing: " << filename << std::endl;
        return;
    }
    for (const auto& edge : edges) {
        file << edge.first.x() << " " << edge.first.y() << " " 
             << edge.second.x() << " " << edge.second.y() << "\n";
    }
}
// Helper: Distance from point p to segment s
double distanceToSegment(const Point& p, const Edge& s) {
    const Point& a = s.first;
    const Point& b = s.second;
    
    double l2 = CGAL::squared_distance(a, b);
    if (l2 == 0.0) return std::sqrt(CGAL::squared_distance(p, a));
    
    // Project p onto line ab, but clamp to segment
    // Vector ab
    double ab_x = b.x() - a.x();
    double ab_y = b.y() - a.y();
    
    // Vector ap
    double ap_x = p.x() - a.x();
    double ap_y = p.y() - a.y();
    
    double t = (ap_x * ab_x + ap_y * ab_y) / l2;
    t = std::max(0.0, std::min(1.0, t));
    
    Point projection(a.x() + t * ab_x, a.y() + t * ab_y);
    return std::sqrt(CGAL::squared_distance(p, projection));
}
// Helper: Distance from point p to a set of edges (min distance)
double distanceToCurve(const Point& p, const std::vector<Edge>& edges) {
    double min_dist = std::numeric_limits<double>::max();
    for (const auto& edge : edges) {
        double d = distanceToSegment(p, edge);
        if (d < min_dist) min_dist = d;
    }
    return min_dist;
}
// Helper: Distance from point p to a set of points (min distance)
double distanceToPoints(const Point& p, const std::vector<Point>& points) {
    double min_dist = std::numeric_limits<double>::max();
    for (const auto& pt : points) {
        double d = std::sqrt(CGAL::squared_distance(p, pt));
        if (d < min_dist) min_dist = d;
    }
    return min_dist;
}
double calculateRMSE(const std::vector<Edge>& edges, const std::vector<Point>& groundTruth) {
    if (edges.empty() || groundTruth.empty()) return -1.0;
    
    double sum_sq_error = 0.0;
    for (const auto& p : groundTruth) {
        double dist = distanceToCurve(p, edges);
        sum_sq_error += dist * dist;
    }
    
    return std::sqrt(sum_sq_error / groundTruth.size());
}
double calculateHausdorff(const std::vector<Edge>& edges, const std::vector<Point>& groundTruth) {
    if (edges.empty() || groundTruth.empty()) return -1.0;
    double max_dist_gt_rec = 0.0;
    for (const auto& p : groundTruth) {
        double dist = distanceToCurve(p, edges);
        if (dist > max_dist_gt_rec) max_dist_gt_rec = dist;
    }
    
    double max_dist_rec_gt = 0.0;
    for (const auto& edge : edges) {
        double d1 = distanceToPoints(edge.first, groundTruth);
        double d2 = distanceToPoints(edge.second, groundTruth);
        max_dist_rec_gt = std::max(max_dist_rec_gt, std::max(d1, d2));
    }
    
    return std::max(max_dist_gt_rec, max_dist_rec_gt);
}
