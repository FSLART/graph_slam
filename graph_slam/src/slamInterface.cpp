#include "graph_slam/slamInterface.h"

#include <cassert>
#include <iostream>

#include "fast_output.h"
#include "g2o/stuff/logger.h"
#include "g2o/types/slam3d/se3quat.h"
#include "graph_optimizer_sparse_online.h"
#include "types_slam2d_online.h"
using namespace std;
using namespace Eigen;

namespace g2o {

G2oSlamInterface::G2oSlamInterface(SparseOptimizerOnline* optimizer)
    : _optimizer(optimizer),
      _firstOptimization(true),
      _nodesAdded(0),
      _incIterations(1),
      _updateGraphEachN(10),
      _batchEveryN(100),
      _lastBatchStep(0),
      _initSolverDone(false) {}

bool G2oSlamInterface::initialize() {
  // allocating the desired solver + testing whether the solver is okay
  if (!_initSolverDone) {
    _initSolverDone = true;
    _optimizer->initSolver(2, _batchEveryN);
  }

  return true;
}

bool G2oSlamInterface::addEdge(int v1Id, int v2Id,
                               const std::vector<double>& measurement,
                               const std::vector<double>& information) {

  size_t oldEdgesSize = _optimizer->edges().size();

  // Allways based on 3 dimensions
  SE2 transf(measurement[0], measurement[1], measurement[2]);
  Eigen::Matrix3d infMat;
  int idx = 0;
  for (int r = 0; r < 3; ++r)
    for (int c = r; c < 3; ++c, ++idx) {
      assert(idx < (int)information.size());
      infMat(r, c) = infMat(c, r) = information[idx];
    }
  // cerr << PVAR(infMat) << endl;
  int doInit = 0;
  SparseOptimizer::Vertex* v1 = _optimizer->vertex(v1Id);
  SparseOptimizer::Vertex* v2 = _optimizer->vertex(v2Id);
  if (!v1) {
    OptimizableGraph::Vertex* v = v1 = addVertex(v1Id);
    _verticesAdded.insert(v);
    doInit = 1;
    ++_nodesAdded;
  }
  if (!v2) {
    OptimizableGraph::Vertex* v = v2 = addVertex(v2Id);
    _verticesAdded.insert(v);
    doInit = 2;
    ++_nodesAdded;
  }
  if (_optimizer->edges().size() == 0) {
    cerr << "FIRST EDGE ";
    if (v1->id() < v2->id()) {
      cerr << "fixing " << v1->id() << endl;
      v1->setFixed(true);
    } else {
      cerr << "fixing " << v2->id() << endl;
      v2->setFixed(true);
    }
  }
  OnlineEdgeSE2* e = new OnlineEdgeSE2;
  e->vertices()[0] = v1;
  e->vertices()[1] = v2;
  e->setMeasurement(transf);
  e->setInformation(infMat);
  _optimizer->addEdge(e);
  _edgesAdded.insert(e);
  if (doInit) {
    OptimizableGraph::Vertex* from =
        static_cast<OptimizableGraph::Vertex*>(e->vertices()[0]);
    OptimizableGraph::Vertex* to =
        static_cast<OptimizableGraph::Vertex*>(e->vertices()[1]);
    switch (doInit) {
      case 1:  // initialize v1 from v2
      {
        HyperGraph::VertexSet toSet;
        toSet.insert(to);
        if (e->initialEstimatePossible(toSet, from) > 0.) {
          e->initialEstimate(toSet, from);
        }
        break;
      }
      case 2: {
        HyperGraph::VertexSet fromSet;
        fromSet.insert(from);
        if (e->initialEstimatePossible(fromSet, to) > 0.) {
          e->initialEstimate(fromSet, to);
        }
        break;
      }
      default:
        cerr << "doInit wrong value\n";
    }
  }

  if (oldEdgesSize == 0) {
    _optimizer->jacobianWorkspace().allocate();
  }

  return true;
}

bool G2oSlamInterface::fixNode(const std::vector<int>& nodes) {
  for (size_t i = 0; i < nodes.size(); ++i) {
    OptimizableGraph::Vertex* v = _optimizer->vertex(nodes[i]);
    if (v) v->setFixed(true);
  }
  return true;
}

bool G2oSlamInterface::solveState() {
  SolveResult state = solve();
  return state != ERROR;
}

OptimizableGraph::Vertex* G2oSlamInterface::addVertex(int id) {
  OnlineVertexSE2* v = new OnlineVertexSE2;
  v->setId(id);  // estimate will be set later when the edge is added
  _optimizer->addVertex(v);
  return v;
}

void G2oSlamInterface::setUpdateGraphEachN(int n) { _updateGraphEachN = n; }

G2oSlamInterface::SolveResult G2oSlamInterface::solve() {
  if (_nodesAdded >= _updateGraphEachN) {
    // decide on batch step or normal step
    _optimizer->batchStep = false;
    if ((int)_optimizer->vertices().size() - _lastBatchStep >= _batchEveryN) {
      _lastBatchStep = _optimizer->vertices().size();
      _optimizer->batchStep = true;
    }

    if (_firstOptimization) {
      if (!_optimizer->initializeOptimization()) {
        cerr << "initialization failed" << endl;
        return ERROR;
      }
    } else {
      if (!_optimizer->updateInitialization(_verticesAdded, _edgesAdded)) {
        cerr << "updating initialization failed" << endl;
        return ERROR;
      }
    }

    int currentIt = _optimizer->optimize(_incIterations, !_firstOptimization);
    (void)currentIt;
    _firstOptimization = false;
    _nodesAdded = 0;
    _verticesAdded.clear();
    _edgesAdded.clear();
    if (_optimizer->batchStep) return SOLVED_BATCH;
    return SOLVED;
  }

  return NOOP;
}

void G2oSlamInterface::setBatchSolveEachN(int n) { _batchEveryN = n; }

}  // namespace g2o
