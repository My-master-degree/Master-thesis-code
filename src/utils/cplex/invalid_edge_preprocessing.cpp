#include "utils/cplex/compact_model.hpp"
#include "utils/cplex/preprocessing_compact_model.hpp"
#include "utils/cplex/invalid_edge_preprocessing.hpp"

using namespace utils::cplex;

Invalid_edge_preprocessing::Invalid_edge_preprocessing (Compact_model& compact_model) : Preprocessing_compact_model (compact_model) {}

void Invalid_edge_preprocessing::add () {
  for (pair<int, Vertex> p : compact_model.all){
    int i = p.first;
    for (pair<int, Vertex> p1 : compact_model.all){
      int j = p1.first;
      if (compact_model.gvrp_instance.distances[i][j] * compact_model.gvrp_instance.vehicleFuelConsumptionRate > compact_model.gvrp_instance.vehicleFuelCapacity || compact_model.gvrp_instance.distances[i][j] / compact_model.gvrp_instance.vehicleAverageSpeed > compact_model.gvrp_instance.timeLimit)
        for (unsigned int k = 0; k < compact_model.gvrp_instance.customers.size(); k++)
          compact_model.model.add(compact_model.x[k][i][j] == 0);
    }
  }
}
