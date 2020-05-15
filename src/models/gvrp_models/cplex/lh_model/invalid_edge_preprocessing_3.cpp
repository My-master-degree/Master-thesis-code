#include "models/gvrp_models/cplex/lh_model/lh_model.hpp"
#include "models/gvrp_models/cplex/lh_model/preprocessing.hpp"
#include "models/gvrp_models/cplex/lh_model/invalid_edge_preprocessing_3.hpp"
#include "utils/util.hpp"

#include <iostream>
#include <list>
#include <string>

using namespace std;
using namespace utils;
using namespace models::gvrp_models::cplex::lh_model;

Invalid_edge_preprocessing_3::Invalid_edge_preprocessing_3 (LH_model& lh_model) : Preprocessing (lh_model) {
  if (lh_model.instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 3' only applies for metric instances");
}

void Invalid_edge_preprocessing_3::add () {
  list<pair<int, int>> edges = get_invalid_edges_3(lh_model.instance, *lh_model.gvrp_afs_tree);
  for (const auto& [i, j] : edges)
    lh_model.model.add(lh_model.x[lh_model.customersC0Indexes[i]][lh_model.customersC0Indexes[j]] == 0);
}
