#include "utils/cplex/compact_model.hpp"
#include "utils/cplex/preprocessing_compact_model.hpp"
#include "utils/cplex/invalid_edge_preprocessing_2.hpp"
#include "utils/util.hpp"

#include <iostream>
#include <map>

using namespace std;
using namespace utils;
using namespace utils::cplex;

Invalid_edge_preprocessing_2::Invalid_edge_preprocessing_2 (Compact_model& compact_model) : Preprocessing_compact_model (compact_model) {}

void Invalid_edge_preprocessing_2::add () {
  map<int, double> customersEnergyUBs = calculateCustomersEnergyUB(compact_model);
  for (Vertex customer : compact_model.gvrp_instance.customers) {
    int i = customer.id;
    double customerEnergyUB = customersEnergyUBs[i];
    for (pair<int, Vertex> p : compact_model.all) {
      int j = p.first;
      if (compact_model.gvrp_instance.distances[i][j] * compact_model.gvrp_instance.vehicleFuelConsumptionRate > customerEnergyUB)
        for (unsigned int k = 0; k < compact_model.gvrp_instance.customers.size(); k++)
          compact_model.model.add(compact_model.x[k][i][j] == 0);
    }
  }
}
