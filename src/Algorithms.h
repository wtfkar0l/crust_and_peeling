#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include "Utils.h"
#include <vector>

// Base abstrata para algoritmos de reconstrução
class ReconstructionAlgorithm {
public:
    virtual std::vector<Edge> reconstruct(const std::vector<Point>& points) = 0;
    virtual std::string name() const = 0;
    virtual ~ReconstructionAlgorithm() {}
};

class CrustAlgorithm : public ReconstructionAlgorithm {
public:
    std::vector<Edge> reconstruct(const std::vector<Point>& points) override;
    std::string name() const override { return "CRUST"; }
};

class PeelingAlgorithm : public ReconstructionAlgorithm {
public:
    std::vector<Edge> reconstruct(const std::vector<Point>& points) override;
    std::string name() const override { return "Peeling the Longest"; }
};

#endif // ALGORITHMS_H
