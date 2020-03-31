#include "models/cubic_model/cubic_model.hpp"
#include "models/gvrp_feasible_solution_heuristic.hpp"
#include "models/cubic_model/preprocessing_cubic_model.hpp"
#include "models/cubic_model/invalid_edge_preprocessing_2.hpp"
#include "utils/util.hpp"

#include <iostream>
#include <list>

using namespace std;
using namespace models;
using namespace utils;
using namespace models::cubic_model;

Invalid_edge_preprocessing_2::Invalid_edge_preprocessing_2 (Cubic_model& cubic_model) : Preprocessing_cubic_model (cubic_model) {
  if (cubic_model.instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 3' only applies for metric instances");
}

void Invalid_edge_preprocessing_2::add () {
  list<pair<int, int>> edges = get_invalid_edges_2(cubic_model.instance);
  for (const auto& [i, j] : edges)
    for (int k = 0; k < cubic_model.instance.nRoutes; k++) 
      cubic_model.model.add(cubic_model.x[k][i][j] == 0);
}
