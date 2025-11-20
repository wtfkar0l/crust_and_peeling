#include "Algorithms.h"
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Voronoi_diagram_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_traits_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_policies_2.h>
#include <iostream>

// Define specific types for CRUST to avoid ambiguity
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point;
typedef CGAL::Delaunay_triangulation_2<K> Delaunay;
typedef CGAL::Delaunay_triangulation_adaptation_traits_2<Delaunay> AT;
typedef CGAL::Delaunay_triangulation_adaptation_policies_2<Delaunay, AT> AP;
typedef CGAL::Voronoi_diagram_2<Delaunay, AT, AP> Voronoi;

std::vector<Edge> CrustAlgorithm::reconstruct(const std::vector<Point>& points) {
    std::vector<Edge> result_edges;
    if (points.empty()) return result_edges;

    // 1. Computar Diagrama de Voronoi de S
    Delaunay dt;
    dt.insert(points.begin(), points.end());
    
    Voronoi vd(dt);

    // 2. Identificar polos P
    std::vector<Point> poles;
    // Para cada site (ponto de entrada), encontrar os vértices de Voronoi mais distantes em sua célula
    
    // Iterar sobre todos os vértices da triangulação de Delaunay (pontos de entrada)
    for (auto v = dt.finite_vertices_begin(); v != dt.finite_vertices_end(); ++v) {
        Point s = v->point();
        double max_dist_sq = -1.0;
        Point pole;
        bool found_pole = false;

        // Circular ao redor do vértice para encontrar faces incidentes (vértices de Voronoi)
        Delaunay::Face_circulator fc = dt.incident_faces(v);
        if (fc != 0) {
            Delaunay::Face_circulator done = fc;
            do {
                if (!dt.is_infinite(fc)) {
                    Point voronoi_vertex = dt.dual(fc);
                    double dist_sq = CGAL::squared_distance(s, voronoi_vertex);
                    if (dist_sq > max_dist_sq) {
                        max_dist_sq = dist_sq;
                        pole = voronoi_vertex;
                        found_pole = true;
                    }
                }
            } while (++fc != done);
        }
        
        if (found_pole) {
            poles.push_back(pole);
            // Adiciona o polo mais distante. O artigo clássico sugere adicionar ambos (p+ e p-),
            // mas o mais distante é o principal estimador do Eixo Medial.
        }
    }

    // 3. Computar Triangulação de Delaunay de S U P
    std::vector<Point> combined_points = points;
    combined_points.insert(combined_points.end(), poles.begin(), poles.end());

    Delaunay dt_combined;
    dt_combined.insert(combined_points.begin(), combined_points.end());

    // 4. Filtrar arestas: Manter (u, v) se u, v em S e (u, v) em DT(S U P)
    for (auto e = dt_combined.finite_edges_begin(); e != dt_combined.finite_edges_end(); ++e) {
        Delaunay::Vertex_handle v1 = e->first->vertex((e->second + 1) % 3);
        Delaunay::Vertex_handle v2 = e->first->vertex((e->second + 2) % 3);

        Point p1 = v1->point();
        Point p2 = v2->point();

        // Verificar se ambos os pontos pertencem ao conjunto original S
        // Otimização: Em produção, usaríamos um set ou marcação nos vértices.
        // Aqui, comparamos linearmente por simplicidade.
        
        bool p1_is_sample = false;
        bool p2_is_sample = false;

        for(const auto& sp : points) {
            if (sp == p1) p1_is_sample = true;
            if (sp == p2) p2_is_sample = true;
        }

        if (p1_is_sample && p2_is_sample) {
            result_edges.push_back({p1, p2});
        }
    }

    return result_edges;
}
