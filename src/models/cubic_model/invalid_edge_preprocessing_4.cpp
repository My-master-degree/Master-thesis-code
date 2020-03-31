#include "models/vertex.hpp"
#include "models/cubic_model/cubic_model.hpp"
#include "models/cubic_model/preprocessing_cubic_model.hpp"
#include "models/cubic_model/invalid_edge_preprocessing_4.hpp"
#include "utils/util.hpp"
#include "models/gvrp_feasible_solution_heuristic.hpp"

#include <iostream>
#include <map>
#include <list>

using namespace std;
using namespace models;
using namespace utils;
using namespace models::cubic_model;

Invalid_edge_preprocessing_4::Invalid_edge_preprocessing_4 (Cubic_model& cubic_model) : Preprocessing_cubic_model (cubic_model) {
  if (cubic_model.instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 4' only applies for metric instances");
}

void Invalid_edge_preprocessing_4::add () {
  list<pair<int, int>> edges = get_invalid_edges_1(cubic_model.instance);
  for (auto const& [i, j] : edges)
    for (int k = 0; k < cubic_model.instance.nRoutes; k++) 
      cubic_model.model.add(cubic_model.x[k][i][j] == 0);
}
