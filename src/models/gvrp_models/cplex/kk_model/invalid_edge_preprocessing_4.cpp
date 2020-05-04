#include "models/gvrp_models/gvrp_feasible_solution_heuristic.hpp"
#include "models/gvrp_models/cplex/kk_model/kk_model.hpp"
#include "models/gvrp_models/cplex/kk_model/preprocessing.hpp"
#include "models/gvrp_models/cplex/kk_model/invalid_edge_preprocessing_4.hpp"
#include "utils/util.hpp"

#include <list>

using namespace std;
using namespace utils;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex::kk_model;

Invalid_edge_preprocessing_4::Invalid_edge_preprocessing_4 (KK_model& kk_model) : Preprocessing (kk_model) {
  if (kk_model.instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 4' only applies for metric instances");
}

void Invalid_edge_preprocessing_4::add () {
  list<pair<int, int>> edges = get_invalid_edges_4(kk_model.instance, *kk_model.gvrp_afs_tree);
  for (auto const& [i, j] : edges) {
    auto f1 = kk_model.afssF0Indexes.find(i),
         f2 = kk_model.afssF0Indexes.find(j);
    //first is afs and second is customer
    if (f1 != kk_model.afssF0Indexes.end()) 
      for (size_t k = 0; k < kk_model.c0.size(); ++k)
        kk_model.model.add(kk_model.y[kk_model.customersC0Indexes[j]][f1->second][k] == 0);
    //first is customer and second is afs 
    else if (f2 != kk_model.afssF0Indexes.end()) 
      for (size_t k = 0; k < kk_model.c0.size(); ++k)
        kk_model.model.add(kk_model.y[kk_model.customersC0Indexes[i]][f2->second][k] == 0);
  }
}
