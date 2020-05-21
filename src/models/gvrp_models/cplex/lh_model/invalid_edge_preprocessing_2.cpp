#include "models/gvrp_models/cplex/lh_model/lh_model.hpp"
#include "models/gvrp_models/gvrp_feasible_solution_heuristic.hpp"
#include "models/gvrp_models/cplex/lh_model/preprocessing.hpp"
#include "models/gvrp_models/cplex/lh_model/invalid_edge_preprocessing_2.hpp"
#include "utils/util.hpp"

#include <iostream>
#include <list>

using namespace std;
using namespace utils;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex::lh_model;

Invalid_edge_preprocessing_2::Invalid_edge_preprocessing_2 (LH_model& lh_model) : Preprocessing (lh_model) {
  if (lh_model.instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 3' only applies for metric instances");
}

void Invalid_edge_preprocessing_2::add () {
  list<pair<int, int>> edges = get_invalid_edges_2(lh_model.instance);
  for (const auto& [i, j] : edges)
    if (i == lh_model.instance.depot.id)
      lh_model.model.add(lh_model.x[0][lh_model.customersC0Indexes[j]] == 0);
    else
      lh_model.model.add(lh_model.x[lh_model.customersC0Indexes[i]][0] == 0);
}
