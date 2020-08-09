#include "models/gvrp_models/cplex/matheus_model/matheus_model.hpp"
#include "models/gvrp_models/gvrp_feasible_solution_heuristic.hpp"
#include "models/gvrp_models/cplex/matheus_model/preprocessing.hpp"
#include "models/gvrp_models/cplex/matheus_model/invalid_edge_preprocessing_2.hpp"
#include "utils/util.hpp"

#include <iostream>
#include <list>

using namespace std;
using namespace utils;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex::matheus_model;

Invalid_edge_preprocessing_2::Invalid_edge_preprocessing_2 (Matheus_model& matheus_model) : Preprocessing (matheus_model) {
  if (matheus_model.instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 3' only applies for metric instances");
}

void Invalid_edge_preprocessing_2::add () {
  list<pair<int, int>> edges = get_invalid_edges_2(matheus_model.instance, *matheus_model.gvrp_afs_tree);
  matheus_model.nPreprocessings2 = edges.size();
  for (const auto& [i, j] : edges)
    if (i == matheus_model.instance.depot.id)
      matheus_model.model.add(matheus_model.x[0][matheus_model.customersC0Indexes[j]] == 0);
    else
      matheus_model.model.add(matheus_model.x[matheus_model.customersC0Indexes[i]][0] == 0);
}
