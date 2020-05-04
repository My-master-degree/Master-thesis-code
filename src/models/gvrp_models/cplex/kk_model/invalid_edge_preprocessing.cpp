#include "models/gvrp_models/cplex/kk_model/kk_model.hpp"
#include "models/gvrp_models/cplex/kk_model/preprocessing.hpp"
#include "models/gvrp_models/cplex/kk_model/invalid_edge_preprocessing.hpp"
#include "utils/util.hpp"

#include <list> 

using namespace utils;
using namespace std;
using namespace models::gvrp_models::cplex::kk_model;

Invalid_edge_preprocessing::Invalid_edge_preprocessing (KK_model& kk_model) : Preprocessing (kk_model) {}

void Invalid_edge_preprocessing::add () {
  //invalid edges
  list<pair<int, int>> edges = get_invalid_edges_1(kk_model.instance);
  for (auto const& [i, j] : edges) {
    auto c1 = kk_model.customersC0Indexes.find(i),
         c2 = kk_model.customersC0Indexes.find(j);
    auto f1 = kk_model.afssF0Indexes.find(i),
         f2 = kk_model.afssF0Indexes.find(j);
    //both are customers
    if (c1 != kk_model.customersC0Indexes.end() && c2 != kk_model.customersC0Indexes.end()) 
      kk_model.model.add(kk_model.x[c1->second][c2->second] == 0);
    //first is afs and second is customer
    else if (f1 != kk_model.afssF0Indexes.end() && c2 != kk_model.customersC0Indexes.end()) 
      for (size_t i = 0; i < kk_model.c0.size(); ++i)
        kk_model.model.add(kk_model.y[i][f1->second][c2->second] == 0);
    //first is customer and second is afs 
    else if (c1 != kk_model.customersC0Indexes.end() && f2 != kk_model.afssF0Indexes.end()) 
      for (size_t i = 0; i < kk_model.c0.size(); ++i)
        kk_model.model.add(kk_model.y[i][f2->second][c1->second] == 0);
    else 
      for (size_t i_ = 0; i_ < kk_model.c0.size(); ++i_)
        for (size_t j_ = 0; j_ < kk_model.c0.size(); ++j_)
          for (size_t k = 0; k < kk_model.c0.size(); ++k)
          kk_model.model.add(kk_model.y[i_][f1->second][j_] + kk_model.y[j_][f2->second][k] <= 1);
  }
}
