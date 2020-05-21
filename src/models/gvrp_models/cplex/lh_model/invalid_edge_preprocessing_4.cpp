#include "models/gvrp_models/gvrp_feasible_solution_heuristic.hpp"
#include "models/gvrp_models/cplex/lh_model/lh_model.hpp"
#include "models/gvrp_models/cplex/lh_model/preprocessing.hpp"
#include "models/gvrp_models/cplex/lh_model/invalid_edge_preprocessing_4.hpp"
#include "utils/util.hpp"

#include <list>

using namespace std;
using namespace utils;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex::lh_model;

Invalid_edge_preprocessing_4::Invalid_edge_preprocessing_4 (LH_model& lh_model) : Preprocessing (lh_model) {
  if (lh_model.instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 4' only applies for metric instances");
}

void Invalid_edge_preprocessing_4::add () {
  list<pair<int, int>> edges = get_invalid_edges_4 (lh_model.instance, *lh_model.gvrp_afs_tree);
  for (auto const& [i, j] : edges) {
    auto f1 = lh_model.afssF0Indexes.find(i),
         f2 = lh_model.afssF0Indexes.find(j);
    //first is afs and second is customer
    if (f1 != lh_model.afssF0Indexes.end()) 
      for (size_t k = 0; k < lh_model.c0.size(); ++k)
        lh_model.model.add(lh_model.y[lh_model.customersC0Indexes[j]][f1->second][k] == 0);
    //first is customer and second is afs 
    else if (f2 != lh_model.afssF0Indexes.end()) 
      for (size_t k = 0; k < lh_model.c0.size(); ++k)
        lh_model.model.add(lh_model.y[lh_model.customersC0Indexes[i]][f2->second][k] == 0);
  }
}
