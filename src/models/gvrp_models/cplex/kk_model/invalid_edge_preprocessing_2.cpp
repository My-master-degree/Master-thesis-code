#include "models/gvrp_models/cplex/kk_model/kk_model.hpp"
#include "models/gvrp_models/gvrp_feasible_solution_heuristic.hpp"
#include "models/gvrp_models/cplex/kk_model/preprocessing.hpp"
#include "models/gvrp_models/cplex/kk_model/invalid_edge_preprocessing_2.hpp"
#include "utils/util.hpp"

#include <iostream>
#include <list>

using namespace std;
using namespace utils;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex::kk_model;

Invalid_edge_preprocessing_2::Invalid_edge_preprocessing_2 (KK_model& kk_model) : Preprocessing (kk_model) {
  if (kk_model.instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 3' only applies for metric instances");
}

void Invalid_edge_preprocessing_2::add () {
  list<pair<int, int>> edges = get_invalid_edges_2(kk_model.instance);
  for (const auto& [i, j] : edges)
    if (i == kk_model.instance.depot.id)
      kk_model.model.add(kk_model.x[0][kk_model.customersC0Indexes[j]] == 0);
    else
      kk_model.model.add(kk_model.x[kk_model.customersC0Indexes[i]][0] == 0);
}
