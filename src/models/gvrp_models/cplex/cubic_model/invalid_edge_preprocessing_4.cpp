#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp"
#include "models/gvrp_models/cplex/cubic_model/preprocessing.hpp"
#include "models/gvrp_models/cplex/cubic_model/invalid_edge_preprocessing_4.hpp"
#include "utils/util.hpp"

#include <map>
#include <list>

using namespace std;
using namespace utils;
using namespace models::gvrp_models::cplex::cubic_model;

Invalid_edge_preprocessing_4::Invalid_edge_preprocessing_4 (Cubic_model& cubic_model) : Preprocessing (cubic_model) {
  if (cubic_model.instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 4' only applies for metric instances");
}

void Invalid_edge_preprocessing_4::add () {
  list<pair<int, int>> edges = get_invalid_edges_1(cubic_model.instance);
  for (auto const& [i, j] : edges)
    for (int k = 0; k < cubic_model.instance.maxRoutes; k++) 
      cubic_model.model.add(cubic_model.x[k][i][j] == 0);
}
