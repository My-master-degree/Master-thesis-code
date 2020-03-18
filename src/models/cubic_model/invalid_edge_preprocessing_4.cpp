#include "models/vertex.hpp"
#include "models/cubic_model/cubic_model.hpp"
#include "models/cubic_model/preprocessing_cubic_model.hpp"
#include "models/cubic_model/invalid_edge_preprocessing_4.hpp"
#include "utils/util.hpp"
#include "models/gvrp_feasible_solution_heuristic.hpp"

#include <iostream>
#include <map>
#include <list>

using namespace std;
using namespace models;
using namespace utils;
using namespace models::cubic_model;

Invalid_edge_preprocessing_4::Invalid_edge_preprocessing_4 (Cubic_model& cubic_model) : Preprocessing_cubic_model (cubic_model) {
  if (cubic_model.gvrp_instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 4' only applies for metric instances");
}

void Invalid_edge_preprocessing_4::add () {
  if (cubic_model.gvrp_instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 4' only applies for metric instances");
  //create induced graph
  vector<Vertex> f0 = createF0Set (cubic_model.gvrp_instance);
  size_t sf0 = f0.size(),
          r,
          f;
  int k;
  //dijkstra
  vector<size_t> pred (sf0);
  vector<double> costs (sf0);
  gvrpDijkstra(f0, pred, costs, cubic_model.gvrp_instance);
  //get invalid edges
  bool invalidEdge;
  double distance;
  for (const Vertex& i : cubic_model.gvrp_instance.customers)
    for (f = 0; f < sf0; f++) {
      invalidEdge = true;
      if (f0[f].id == cubic_model.gvrp_instance.depot.id || pred[f] != f) 
        for (r = 0; r < sf0; r++) 
          //if the afss f and r are connected 
          if (f0[r].id == cubic_model.gvrp_instance.depot.id || pred[r] != r) {
            distance = cubic_model.gvrp_instance.distances[f0[f].id][i.id] + cubic_model.gvrp_instance.distances[i.id][f0[r].id];
            if (distance * cubic_model.gvrp_instance.vehicleFuelConsumptionRate <= cubic_model.gvrp_instance.vehicleFuelCapacity && (costs[f] + distance + costs[r])/cubic_model.gvrp_instance.vehicleAverageSpeed + f0[f].serviceTime + i.serviceTime + f0[r].serviceTime <= cubic_model.gvrp_instance.timeLimit) { 
              invalidEdge = false;
              break;
            }
          }
      if (invalidEdge) {
        //cout<<"Invalid edge here in "<<i.id<<" "<<j.id<<endl;
        for (k = 0; k < cubic_model.gvrp_instance.nRoutes; k++) {
          cubic_model.model.add(cubic_model.x[k][i.id][f0[f].id] == 0);
          cubic_model.model.add(cubic_model.x[k][f0[f].id][i.id] == 0);
        }
      }
    }
}
