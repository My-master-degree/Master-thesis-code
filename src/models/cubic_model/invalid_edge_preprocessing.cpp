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
      if (cubic_model.instance.distances[i][j] * cubic_model.instance.vehicleFuelConsumptionRate > cubic_model.instance.vehicleFuelCapacity || cubic_model.instance.distances[i][j] / cubic_model.instance.vehicleAverageSpeed > cubic_model.instance.timeLimit)
        for (int k = 0; k < cubic_model.instance.nRoutes; k++)
          cubic_model.model.add(cubic_model.x[k][i][j] == 0);
    }
  }
}
