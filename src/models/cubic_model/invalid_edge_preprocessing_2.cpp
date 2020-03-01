#include "models/cubic_model/cubic_model.hpp"
#include "models/gvrp_feasible_solution_heuristic.hpp"
#include "models/cubic_model/preprocessing_cubic_model.hpp"
#include "models/cubic_model/invalid_edge_preprocessing_2.hpp"
#include "utils/util.hpp"

#include <iostream>
#include <map>

using namespace std;
using namespace models;
using namespace utils;
using namespace models::cubic_model;

Invalid_edge_preprocessing_2::Invalid_edge_preprocessing_2 (Cubic_model& cubic_model) : Preprocessing_cubic_model (cubic_model) {
  if (cubic_model.gvrp_instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 3' only applies for metric instances");
}

void Invalid_edge_preprocessing_2::add () {
  if (cubic_model.gvrp_instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 3' only applies for metric instances");
  Gvrp_feasible_solution_heuristic gvrp_feasible_solution_heuristic (cubic_model.gvrp_instance);
  Gvrp_solution gvrp_solution =  gvrp_feasible_solution_heuristic.run();
  //for each route
  for (const list<Vertex>& route : gvrp_solution.routes) {
    auto second = ++route.begin();
    auto penultimate = --route.rbegin();
    //valid preprocessing
    if (!cubic_model.customers.count(second->id) && !cubic_model.customers.count(penultimate->id)) {
      //get customer
      for (second++; !cubic_model.customers.count(second->id); second++);
      for (size_t k = 0; k < cubic_model.gvrp_instance.customers.size(); k++) {
        cubic_model.model.add(cubic_model.x[k][0][second->id] == 0);
        cubic_model.model.add(cubic_model.x[k][second->id][0] == 0);
      }
    }
  }
}
