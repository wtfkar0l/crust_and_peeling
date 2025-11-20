#include "Utils.h"
#include <fstream>
#include <iostream>
#include <sstream>

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
