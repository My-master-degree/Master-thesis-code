#include "models/gvrp_models/cplex/matheus_model/matheus_model.hpp"
#include "models/gvrp_models/cplex/matheus_model/preprocessing.hpp"
#include "models/gvrp_models/cplex/matheus_model/invalid_edge_preprocessing_3.hpp"
#include "utils/util.hpp"

#include <iostream>
#include <list>
#include <string>

using namespace std;
using namespace utils;
using namespace models::gvrp_models::cplex::matheus_model;

Invalid_edge_preprocessing_3::Invalid_edge_preprocessing_3 (Matheus_model& matheus_model) : Preprocessing (matheus_model) {
  if (matheus_model.instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 3' only applies for metric instances");
}

void Invalid_edge_preprocessing_3::add () {
  list<pair<int, int>> edges = get_invalid_edges_3 (matheus_model.instance, *matheus_model.gvrp_afs_tree);
  for (const auto& [i, j] : edges)
    matheus_model.model.add(matheus_model.x[matheus_model.customersC0Indexes[i]][matheus_model.customersC0Indexes[j]] == 0);
}
