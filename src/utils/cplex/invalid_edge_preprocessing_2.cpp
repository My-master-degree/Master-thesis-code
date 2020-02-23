#include "utils/cplex/compact_model.hpp"
#include "models/gvrp_feasible_solution_heuristic.hpp"
#include "utils/cplex/preprocessing_compact_model.hpp"
#include "utils/cplex/invalid_edge_preprocessing_2.hpp"
#include "utils/util.hpp"

#include <iostream>
#include <map>

using namespace std;
using namespace models;
using namespace utils;
using namespace utils::cplex;

Invalid_edge_preprocessing_2::Invalid_edge_preprocessing_2 (Compact_model& compact_model) : Preprocessing_compact_model (compact_model) {
  if (compact_model.gvrp_instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 3' only applies for metric instances");
}

void Invalid_edge_preprocessing_2::add () {
  if (compact_model.gvrp_instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 3' only applies for metric instances");
  Gvrp_feasible_solution_heuristic gvrp_feasible_solution_heuristic (compact_model.gvrp_instance);
  Gvrp_solution gvrp_solution =  gvrp_feasible_solution_heuristic.run();
  //for each route
  for (const list<Vertex>& route : gvrp_solution.routes) {
    auto second = ++route.begin();
    auto penultimate = --route.rbegin();
    //valid preprocessing
    if (!compact_model.customers.count(second->id) && !compact_model.customers.count(penultimate->id)) {
      //get customer
      for (second++; !compact_model.customers.count(second->id); second++);
      for (size_t k = 0; k < compact_model.gvrp_instance.customers.size(); k++) {
        compact_model.model.add(compact_model.x[k][0][second->id] == 0);
        compact_model.model.add(compact_model.x[k][second->id][0] == 0);
      }
    }
  }
}
