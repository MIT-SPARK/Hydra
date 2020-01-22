//            Copyright Daniel Trebbien 2010.
// Distributed under the Boost Software License, Version 1.0.
//   (See accompanying file LICENSE_1_0.txt or the copy at
//         http://www.boost.org/LICENSE_1_0.txt)

#pragma once

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/one_bit_color_map.hpp>
#include <boost/graph/stoer_wagner_min_cut.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/typeof/typeof.hpp>
#include <cassert>
#include <cstddef>
#include <cstdlib>
#include <iostream>

#include <glog/logging.h>

namespace kimera {

class StoerWagnerMinCut {
 public:
  struct Edge {
    unsigned long first;
    unsigned long second;
  };
  typedef std::vector<Edge> Edges;

  typedef boost::adjacency_list<boost::vecS,
                                boost::vecS,
                                boost::undirectedS,
                                boost::no_property,
                                boost::property<boost::edge_weight_t, int> >
      UndirectedGraph;
  typedef boost::property_map<UndirectedGraph, boost::edge_weight_t>::type
      WeightMap;
  typedef boost::property_traits<WeightMap>::value_type WeightType;
  typedef std::vector<WeightType> Weights;

  StoerWagnerMinCut(const size_t& n_vertices,
                    const Edges& edges,
                    const Weights& weights,
                    const bool& verbose = false)
      : n_vertices_(n_vertices),
        edges_(edges),
        weights_(weights),
        verbose_(verbose) {}

  bool solve() {
    CHECK_EQ(edges_.size(), weights_.size());

    // Construct the graph.
    UndirectedGraph g(edges_.begin(),
                      edges_.end(),
                      weights_.begin(),
                      n_vertices_,
                      edges_.size());

    // Define a property map, `parities`, that will store a boolean value for
    // each vertex. Vertices that have the same parity after
    // `stoer_wagner_min_cut` runs are on the same side of the min-cut.
    BOOST_AUTO(parities,
               boost::make_one_bit_color_map(num_vertices(g),
                                             get(boost::vertex_index, g)));

    // Run the Stoer-Wagner algorithm to obtain the min-cut weight. `parities`
    // is also filled in.
    int w = boost::stoer_wagner_min_cut(
        g, get(boost::edge_weight, g), boost::parity_map(parities));

    if (verbose_) {
      LOG(INFO) << "The min-cut weight of G is " << w << ".\n"
                << "One set of vertices consists of:";
      size_t i;
      for (i = 0; i < num_vertices(g); ++i) {
        if (get(parities, i)) {
          LOG(INFO) << i;
        }
      }

      LOG(INFO) << "The other set of vertices consists of:\n";
      for (i = 0; i < num_vertices(g); ++i) {
        if (!get(parities, i)) {
          LOG(INFO) << i;
        }
      }
    }

    return true;
  }

 private:
  size_t n_vertices_;
  Edges edges_;
  Weights weights_;
  bool verbose_ = true;
};

}  // namespace kimera
