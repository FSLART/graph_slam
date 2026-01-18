#include "graph_slam/associationSolver.hpp"

#include <stdexcept>

// ================== Backend interface =======================================

class AssociationSolver::AssociationBackend
{
public:
    virtual ~AssociationBackend() = default;
    virtual void associate(const lart_msgs::msg::ConeArray &observations) = 0;
};

// ================== Nearest Neighbor backend =================================

class NearestNeighborBackend : public AssociationSolver::AssociationBackend
{
public:
    void associate(const lart_msgs::msg::ConeArray &observations) override
    {
        // TODO: implement NN association against your current map / graph
        // Example structure:
        //  for each observation:
        //    find map cone with minimum Euclidean distance
        (void)observations;
    }
};

// ================== Mahalanobis backend ======================================

class MahalanobisBackend : public AssociationSolver::AssociationBackend
{
public:
    void associate(const lart_msgs::msg::ConeArray &observations) override
    {
        // TODO: implement Mahalanobis distance-based association
        // d^2 = (z - h(x))^T S^{-1} (z - h(x))
        (void)observations;
    }
};

// ================== AssociationSolver front-end ==============================

AssociationSolver::AssociationSolver(int mode)
{
    switch (mode)
    {
    case 0:
        backend_ = std::make_unique<NearestNeighborBackend>();
        break;
    case 1:
        backend_ = std::make_unique<MahalanobisBackend>();
        break;
    default:
        throw std::invalid_argument("Unknown association mode");
    }
}

AssociationSolver::~AssociationSolver() = default;

void AssociationSolver::associate(const lart_msgs::msg::ConeArray &observations)
{
    if (backend_)
        backend_->associate(observations);
}