#include "models/gvrp_models/cplex/emh_model/emh_model.hpp"
#include "models/gvrp_models/cplex/emh_model/preprocessing.hpp"
#include "models/gvrp_models/cplex/emh_model/invalid_edge_preprocessing_3.hpp"
#include "utils/util.hpp"

#include <iostream>
#include <list>
#include <string>

using namespace std;
using namespace utils;
using namespace models::gvrp_models::cplex::emh_model;

Invalid_edge_preprocessing_3::Invalid_edge_preprocessing_3 (EMH_model& emh_model) : Preprocessing (emh_model) {
  if (emh_model.instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 3' only applies for metric instances");
}

void Invalid_edge_preprocessing_3::add () {
  list<pair<int, int>> edges = get_invalid_edges_3(emh_model.instance, *emh_model.gvrp_afs_tree);
  for (const auto& [i, j] : edges)
    emh_model.model.add(emh_model.x[i][j] == 0);
}