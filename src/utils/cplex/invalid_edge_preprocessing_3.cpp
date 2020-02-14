#include "models/vertex.hpp"
#include "utils/cplex/compact_model.hpp"
#include "utils/cplex/preprocessing_compact_model.hpp"
#include "utils/cplex/invalid_edge_preprocessing_3.hpp"
#include "utils/util.hpp"

#include <iostream>
#include <map>
#include <vertex>

using namespace std;
using namespace utils;
using namespace utils::cplex;

Invalid_edge_preprocessing_3::Invalid_edge_preprocessing_3 (Compact_model& compact_model) : Preprocessing_compact_model (compact_model) {}

void Invalid_edge_preprocessing_3::add () {
  vector<Vertex> f0 = creatF0Set(compact_model.gvrp_instance);
  gvrpDijkstra(f0, 
     
}
