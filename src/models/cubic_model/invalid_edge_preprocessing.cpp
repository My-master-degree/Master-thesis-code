#include "models/cubic_model/cubic_model.hpp"
#include "models/cubic_model/preprocessing_cubic_model.hpp"
#include "models/cubic_model/invalid_edge_preprocessing.hpp"

using namespace models::cubic_model;

Invalid_edge_preprocessing::Invalid_edge_preprocessing (Cubic_model& cubic_model) : Preprocessing_cubic_model (cubic_model) {}

void Invalid_edge_preprocessing::add () {
  for (pair<int, Vertex> p : cubic_model.all){
    int i = p.first;
    for (pair<int, Vertex> p1 : cubic_model.all){
      int j = p1.first;
      if (cubic_model.gvrp_instance.distances[i][j] * cubic_model.gvrp_instance.vehicleFuelConsumptionRate > cubic_model.gvrp_instance.vehicleFuelCapacity || cubic_model.gvrp_instance.distances[i][j] / cubic_model.gvrp_instance.vehicleAverageSpeed > cubic_model.gvrp_instance.timeLimit)
        for (int k = 0; k < cubic_model.gvrp_instance.nRoutes; k++)
          cubic_model.model.add(cubic_model.x[k][i][j] == 0);
    }
  }
}
