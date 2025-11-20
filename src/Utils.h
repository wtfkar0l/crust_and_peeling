#ifndef UTILS_H
#define UTILS_H

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Voronoi_diagram_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_traits_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_policies_2.h>
#include <vector>
#include <string>
#include <utility>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point;
typedef K::Segment_2 Segment;
typedef CGAL::Delaunay_triangulation_2<K> Delaunay;

// Definições de tipos para Voronoi
typedef CGAL::Delaunay_triangulation_adaptation_traits_2<Delaunay> AT;
typedef CGAL::Delaunay_triangulation_adaptation_policies_2<Delaunay, AT> AP;
typedef CGAL::Voronoi_diagram_2<Delaunay, AT, AP> Voronoi;

// Tipo de aresta para saída (par de pontos)
typedef std::pair<Point, Point> Edge;

// Função para carregar pontos de um arquivo
std::vector<Point> loadPoints(const std::string& filename);

// Função para salvar arestas em um arquivo (ex: para visualização)
void saveEdges(const std::vector<Edge>& edges, const std::string& filename);

// Métricas
double calculateRMSE(const std::vector<Edge>& edges, const std::vector<Point>& groundTruth);

#endif // UTILS_H
