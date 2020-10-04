#include "models/gvrp_models/cplex/matheus_model_3/matheus_model_3.hpp"
#include "models/gvrp_models/gvrp_feasible_solution_heuristic.hpp"
#include "models/gvrp_models/cplex/matheus_model_3/preprocessing.hpp"
#include "models/gvrp_models/cplex/matheus_model_3/invalid_edge_preprocessing_2.hpp"
#include "utils/util.hpp"

#include <iostream>
#include <list>

using namespace std;
using namespace utils;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex::matheus_model_3;

Invalid_edge_preprocessing_2::Invalid_edge_preprocessing_2 (Matheus_model_3& matheus_model_3) : Preprocessing (matheus_model_3) {
  if (matheus_model_3.instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 3' only applies for metric instances");
}

void Invalid_edge_preprocessing_2::add () {
  list<pair<int, int>> edges = get_invalid_edges_2(matheus_model_3.instance, *matheus_model_3.gvrp_afs_tree);
  matheus_model_3.nPreprocessings2 = edges.size();
  int depotId = matheus_model_3.instance.depot.id;
  for (const auto& [i, j] : edges)
    if (i == depotId) {
      matheus_model_3.model.add(matheus_model_3.x[depotId][j] == 0);
    } else {
      matheus_model_3.model.add(matheus_model_3.x[i][depotId] == 0);
    }
}
