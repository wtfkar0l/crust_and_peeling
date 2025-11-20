#include "Algorithms.h"
#include "Utils.h"
#include <iostream>
#include <string>
#include <memory>
#include <chrono>

int main(int argc, char* argv[]) {
    if (argc < 4) {
        std::cout << "Usage: " << argv[0] << " <input_file> <output_file> <algorithm_type (crust|peeling)>" << std::endl;
        return 1;
    }

    std::string inputFile = argv[1];
    std::string outputFile = argv[2];
    std::string algoType = argv[3];

    std::cout << "Carregando pontos de " << inputFile << "..." << std::endl;
    std::vector<Point> points = loadPoints(inputFile);
    if (points.empty()) {
        std::cerr << "Nenhum ponto carregado." << std::endl;
        return 1;
    }
    std::cout << "Carregados " << points.size() << " pontos." << std::endl;

    std::unique_ptr<ReconstructionAlgorithm> algorithm;
    if (algoType == "crust") {
        algorithm = std::make_unique<CrustAlgorithm>();
    } else if (algoType == "peeling") {
        algorithm = std::make_unique<PeelingAlgorithm>();
    } else {
        std::cerr << "Tipo de algoritmo desconhecido: " << algoType << std::endl;
        return 1;
    }

    std::cout << "Executando " << algorithm->name() << "..." << std::endl;
    auto start = std::chrono::high_resolution_clock::now();
    
    std::vector<Edge> edges = algorithm->reconstruct(points);
    
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;

    std::cout << "Reconstrução completa em " << elapsed.count() << " ms." << std::endl;
    std::cout << "Geradas " << edges.size() << " arestas." << std::endl;

    std::cout << "Salvando arestas em " << outputFile << "..." << std::endl;
    saveEdges(edges, outputFile);

    return 0;
}
