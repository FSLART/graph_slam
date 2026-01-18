#ifndef ASSOCIATION_SOLVER_H_
#define ASSOCIATION_SOLVER_H_

#include <memory>
#include "lart_msgs/msg/cone_array.hpp"

class AssociationSolver
{
public:
    AssociationSolver(int mode);
    ~AssociationSolver();

    void associate(const lart_msgs::msg::ConeArray &observations);

    class AssociationBackend;

private:
    std::unique_ptr<AssociationBackend> backend_;
};

#endif