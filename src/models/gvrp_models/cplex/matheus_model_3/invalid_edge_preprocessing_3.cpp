#include "models/gvrp_models/cplex/matheus_model_3/matheus_model_3.hpp"
#include "models/gvrp_models/cplex/matheus_model_3/preprocessing.hpp"
#include "models/gvrp_models/cplex/matheus_model_3/invalid_edge_preprocessing_3.hpp"
#include "utils/util.hpp"

#include <iostream>
#include <list>
#include <string>

using namespace std;
using namespace utils;
using namespace models::gvrp_models::cplex::matheus_model_3;

Invalid_edge_preprocessing_3::Invalid_edge_preprocessing_3 (Matheus_model_3& matheus_model_3) : Preprocessing (matheus_model_3) {
  if (matheus_model_3.instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 3' only applies for metric instances");
}

void Invalid_edge_preprocessing_3::add () {
  list<pair<int, int>> edges = get_invalid_edges_3(matheus_model_3.instance, *matheus_model_3.gvrp_afs_tree);
  matheus_model_3.nPreprocessings3 = edges.size();
  for (const auto& [i, j] : edges)
    matheus_model_3.model.add(matheus_model_3.x[i][j] == 0);
}
