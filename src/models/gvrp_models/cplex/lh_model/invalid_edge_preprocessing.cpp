#include "models/gvrp_models/cplex/lh_model/lh_model.hpp"
#include "models/gvrp_models/cplex/lh_model/preprocessing.hpp"
#include "models/gvrp_models/cplex/lh_model/invalid_edge_preprocessing.hpp"
#include "utils/util.hpp"

#include <list> 

using namespace utils;
using namespace std;
using namespace models::gvrp_models::cplex::lh_model;

Invalid_edge_preprocessing::Invalid_edge_preprocessing (LH_model& lh_model) : Preprocessing (lh_model) {}

void Invalid_edge_preprocessing::add () {
  //invalid edges
  list<pair<int, int>> edges = get_invalid_edges_1(lh_model.instance);
  for (auto const& [i, j] : edges) {
    auto c1 = lh_model.customersC0Indexes.find(i),
         c2 = lh_model.customersC0Indexes.find(j);
    auto f1 = lh_model.afssF0Indexes.find(i),
         f2 = lh_model.afssF0Indexes.find(j);
    //both are customers
    if (c1 != lh_model.customersC0Indexes.end() && c2 != lh_model.customersC0Indexes.end()) 
      lh_model.model.add(lh_model.x[c1->second][c2->second] == 0);
    //first is afs and second is customer
    else if (f1 != lh_model.afssF0Indexes.end() && c2 != lh_model.customersC0Indexes.end()) 
      for (size_t i = 0; i < lh_model.c0.size(); ++i)
        lh_model.model.add(lh_model.y[i][f1->second][c2->second] == 0);
    //first is customer and second is afs 
    else if (c1 != lh_model.customersC0Indexes.end() && f2 != lh_model.afssF0Indexes.end()) 
      for (size_t i = 0; i < lh_model.c0.size(); ++i)
        lh_model.model.add(lh_model.y[i][f2->second][c1->second] == 0);
    else 
      for (size_t i_ = 0; i_ < lh_model.c0.size(); ++i_)
        for (size_t j_ = 0; j_ < lh_model.c0.size(); ++j_)
          for (size_t k = 0; k < lh_model.c0.size(); ++k)
          lh_model.model.add(lh_model.y[i_][f1->second][j_] + lh_model.y[j_][f2->second][k] <= 1);
  }
}
