#ifndef GVRP_AFS_TREE_HPP_
#define GVRP_AFS_TREE_HPP_

#include "models/gvrp_models/gvrp_instance.hpp"

namespace models {
  namespace gvrp_models {
    class Gvrp_afs_tree {
      public:
        explicit Gvrp_afs_tree (const Gvrp_instance& gvrp_instance);
        vector<const Vertex *> f0; 
        vector<double> times;
        vector<size_t> pred;
        vector<double> fuels;
        vector<vector<double>> pairCosts;
        vector<vector<double>> pairTimes;
        vector<vector<size_t>> pairPreds;
    };
  }
}


#endif
