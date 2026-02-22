
#ifndef G2O_SLAM_INTERFACE_H
#define G2O_SLAM_INTERFACE_H

#include <map>
#include <vector>

#include "g2o/core/optimizable_graph.h"
#include "g2o_interactive_api.h"
#include "slam_parser/interface/abstract_slam_interface.h"

namespace g2o {

class SparseOptimizerOnline;

class G2O_INTERACTIVE_API G2oSlamInterface
    : public SlamParser::AbstractSlamInterface {
 public:
  enum SolveResult { SOLVED, SOLVED_BATCH, NOOP, ERROR };

 public:
  G2oSlamInterface(SparseOptimizerOnline* optimizer);

  bool initialize();

  bool addEdge(const std::string& tag, int id, int dimension, int v1, int v2,
               const std::vector<double>& measurement,
               const std::vector<double>& information);

  bool fixNode(const std::vector<int>& nodes);

  bool queryState(const std::vector<int>& nodes);

  bool solveState();

  SolveResult solve();

  int updatedGraphEachN() const { return _updateGraphEachN; }
  void setUpdateGraphEachN(int n);

  int batchSolveEachN() const { return _batchEveryN; }
  void setBatchSolveEachN(int n);

  SparseOptimizerOnline* optimizer() { return _optimizer; }

 protected:
  SparseOptimizerOnline* _optimizer;
  bool _firstOptimization;
  int _nodesAdded;
  int _incIterations;
  int _updateGraphEachN;
  int _batchEveryN;
  int _lastBatchStep;
  bool _initSolverDone;

  HyperGraph::VertexSet _verticesAdded;
  HyperGraph::EdgeSet _edgesAdded;

  OptimizableGraph::Vertex* addVertex(int dimension, int id);
  bool printVertex(OptimizableGraph::Vertex* v);
};

}  // namespace g2o

#endif