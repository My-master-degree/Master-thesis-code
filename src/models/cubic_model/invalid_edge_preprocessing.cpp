#include "models/cubic_model/cubic_model.hpp"
#include "models/cubic_model/preprocessing_cubic_model.hpp"
#include "models/cubic_model/invalid_edge_preprocessing.hpp"
#include "utils/util.hpp"

using namespace models::cubic_model;
using namespace utils;

Invalid_edge_preprocessing::Invalid_edge_preprocessing (Cubic_model& cubic_model) : Preprocessing_cubic_model (cubic_model) {}

void Invalid_edge_preprocessing::add () {
  list<pair<int, int>> edges = get_invalid_edges_1(cubic_model.instance);
  for (auto const& [i, j] : edges)
    for (int k = 0; k < cubic_model.instance.nRoutes; k++)
      cubic_model.model.add(cubic_model.x[k][i][j] == 0);
}
