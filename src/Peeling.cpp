#include "Algorithms.h"
#include <CGAL/Delaunay_triangulation_2.h>
#include <set>
#include <map>
#include <algorithm>
#include <iostream>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point;
typedef CGAL::Delaunay_triangulation_2<K> Delaunay;

// Auxiliar para canonicalizar arestas para armazenamento em set
Edge make_edge(const Point& p1, const Point& p2) {
    if (p1 < p2) return {p1, p2};
    return {p2, p1};
}

std::vector<Edge> PeelingAlgorithm::reconstruct(const std::vector<Point>& points) {
    std::vector<Edge> result_edges;
    if (points.empty()) return result_edges;

    Delaunay dt;
    dt.insert(points.begin(), points.end());

    // Armazenar arestas ativas. Usamos um set para rastrear a existência.
    std::set<Edge> active_edges;

    // Inicializar com todas as arestas finitas
    for (auto e = dt.finite_edges_begin(); e != dt.finite_edges_end(); ++e) {
        Point p1 = e->first->vertex((e->second + 1) % 3)->point();
        Point p2 = e->first->vertex((e->second + 2) % 3)->point();
        active_edges.insert(make_edge(p1, p2));
    }

    // Fase 1: Peeling Global
    // Para cada triângulo, remover a aresta mais longa
    for (auto f = dt.finite_faces_begin(); f != dt.finite_faces_end(); ++f) {
        Point p0 = f->vertex(0)->point();
        Point p1 = f->vertex(1)->point();
        Point p2 = f->vertex(2)->point();

        double d01 = CGAL::squared_distance(p0, p1);
        double d12 = CGAL::squared_distance(p1, p2);
        double d20 = CGAL::squared_distance(p2, p0);

        Edge e_remove;
        if (d01 >= d12 && d01 >= d20) {
            e_remove = make_edge(p0, p1);
        } else if (d12 >= d01 && d12 >= d20) {
            e_remove = make_edge(p1, p2);
        } else {
            e_remove = make_edge(p2, p0);
        }

        active_edges.erase(e_remove);
    }

    // Fase 2: Refinamento Local
    // Construir lista de adjacência
    std::map<Point, std::vector<Point>> adj;
    for (const auto& edge : active_edges) {
        adj[edge.first].push_back(edge.second);
        adj[edge.second].push_back(edge.first);
    }

    bool changed = true;
    while (changed) {
        changed = false;
        // Iterar sobre todos os vértices para verificar restrições de grau
        
        for (auto& entry : adj) {
            Point p = entry.first;
            std::vector<Point>& neighbors = entry.second;

            if (neighbors.size() > 2) {
                // Encontrar a aresta incidente mais longa
                double max_dist = -1.0;
                Point best_neighbor;
                int best_idx = -1;

                for (int i = 0; i < neighbors.size(); ++i) {
                    double d = CGAL::squared_distance(p, neighbors[i]);
                    if (d > max_dist) {
                        max_dist = d;
                        best_neighbor = neighbors[i];
                        best_idx = i;
                    }
                }

                // Remover aresta (p, best_neighbor)
                if (best_idx != -1) {
                    // Remover da lista de p
                    neighbors.erase(neighbors.begin() + best_idx);
                    
                    // Remover da lista do vizinho
                    auto& n_neighbors = adj[best_neighbor];
                    for (int i = 0; i < n_neighbors.size(); ++i) {
                        if (n_neighbors[i] == p) {
                            n_neighbors.erase(n_neighbors.begin() + i);
                            break;
                        }
                    }
                    
                    // Remover do set de arestas ativas
                    active_edges.erase(make_edge(p, best_neighbor));
                    
                    changed = true;
                }
            }
        }
    }

    // Coletar resultado
    result_edges.assign(active_edges.begin(), active_edges.end());
    return result_edges;
}
