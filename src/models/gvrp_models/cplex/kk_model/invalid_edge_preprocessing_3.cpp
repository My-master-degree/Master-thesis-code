#include "models/gvrp_models/cplex/kk_model/kk_model.hpp"
#include "models/gvrp_models/cplex/kk_model/preprocessing.hpp"
#include "models/gvrp_models/cplex/kk_model/invalid_edge_preprocessing_3.hpp"
#include "utils/util.hpp"

#include <iostream>
#include <list>
#include <string>

using namespace std;
using namespace utils;
using namespace models::gvrp_models::cplex::kk_model;

Invalid_edge_preprocessing_3::Invalid_edge_preprocessing_3 (KK_model& kk_model) : Preprocessing (kk_model) {
  if (kk_model.instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 3' only applies for metric instances");
}

void Invalid_edge_preprocessing_3::add () {
  list<pair<int, int>> edges = get_invalid_edges_3(kk_model.instance, *kk_model.gvrp_afs_tree);
  for (const auto& [i, j] : edges)
    kk_model.model.add(kk_model.x[kk_model.customersC0Indexes[i]][kk_model.customersC0Indexes[j]] == 0);
}
