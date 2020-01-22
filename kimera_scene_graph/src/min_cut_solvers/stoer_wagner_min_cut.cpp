#include "kimera_scene_graph/min_cut_solvers/stoer_wagner_min_cut.h"

#include <glog/logging.h>

namespace kimera {}

// A graphic of the min-cut is available at
// <http://www.boost.org/doc/libs/release/libs/graph/doc/stoer_wagner_imgs/stoer_wagner.cpp.gif>
int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  // define the 16 edges of the graph. {3, 4} means an undirected edge between
  // vertices 3 and 4.
  kimera::StoerWagnerMinCut::Edges edges = {{3, 4},
                                            {3, 6},
                                            {3, 5},
                                            {0, 4},
                                            {0, 1},
                                            {0, 6},
                                            {0, 7},
                                            {0, 5},
                                            {0, 2},
                                            {4, 1},
                                            {1, 6},
                                            {1, 5},
                                            {6, 7},
                                            {7, 5},
                                            {5, 2},
                                            {3, 4}};

  // for each of the 16 edges, define the associated edge weight. ws[i] is the
  // weight for the edge that is described by edges[i].
  kimera::StoerWagnerMinCut::Weights weights = {
      0, 3, 1, 3, 1, 2, 6, 1, 8, 1, 1, 80, 2, 1, 1, 4};

  // 8 is the number of vertices.
  kimera::StoerWagnerMinCut algo (8, edges, weights, true);
  algo.solve();
}
