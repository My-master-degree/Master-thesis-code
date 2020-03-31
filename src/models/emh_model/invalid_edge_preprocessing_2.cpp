#include "models/emh_model/emh_model.hpp"
#include "models/gvrp_feasible_solution_heuristic.hpp"
#include "models/emh_model/preprocessing_emh_model.hpp"
#include "models/emh_model/invalid_edge_preprocessing_2.hpp"
#include "utils/util.hpp"

#include <iostream>
#include <list>

using namespace std;
using namespace models;
using namespace utils;
using namespace models::emh_model;

Invalid_edge_preprocessing_2::Invalid_edge_preprocessing_2 (EMH_model& emh_model) : Preprocessing_emh_model (emh_model) {
  if (emh_model.instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 3' only applies for metric instances");
}

void Invalid_edge_preprocessing_2::add () {
  list<pair<int, int>> edges = get_invalid_edges_2(emh_model.instance);
  for (const auto& [i, j] : edges)
    emh_model.model.add(emh_model.x[i][j] == 0);
}
