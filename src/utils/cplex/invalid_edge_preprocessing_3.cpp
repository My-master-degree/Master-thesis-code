#include "models/vertex.hpp"
#include "utils/cplex/compact_model.hpp"
#include "utils/cplex/preprocessing_compact_model.hpp"
#include "utils/cplex/invalid_edge_preprocessing_3.hpp"
#include "utils/util.hpp"
#include "models/gvrp_feasible_solution_heuristic.hpp"

#include <iostream>
#include <map>
#include <list>

using namespace std;
using namespace models;
using namespace utils;
using namespace utils::cplex;

Invalid_edge_preprocessing_3::Invalid_edge_preprocessing_3 (Compact_model& compact_model) : Preprocessing_compact_model (compact_model) {
  if (compact_model.gvrp_instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 3' only applies for metric instances");
}

void Invalid_edge_preprocessing_3::add () {
  if (compact_model.gvrp_instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 3' only applies for metric instances");
  //create induced graph
  vector<Vertex> f0 = createF0Set (compact_model.gvrp_instance);
  size_t sf0 = f0.size(),
          r,
          f,
          k;
  //dijkstra
  vector<size_t> pred (sf0);
  vector<double> costs (sf0);
  gvrpDijkstra(f0, pred, costs, compact_model.gvrp_instance);
  //get invalid edges
  bool invalidEdge;
  double distance;
  for (const Vertex& i : compact_model.gvrp_instance.customers) 
    for (const Vertex& j : compact_model.gvrp_instance.customers) {
      invalidEdge = true;
      //check if edge (i, j) can be feasible
      for (f = 0; f < sf0; f++) 
        if (f0[f].id == compact_model.gvrp_instance.depot.id || pred[f] != f) 
          for (r = 0; r < sf0; r++) 
            //if the afss f and r are connected 
            if (f0[r].id == compact_model.gvrp_instance.depot.id || pred[r] != r) {
              distance = compact_model.gvrp_instance.distances[f0[f].id][i.id] + compact_model.gvrp_instance.distances[i.id][j.id] + compact_model.gvrp_instance.distances[j.id][f0[r].id];
              if (distance * compact_model.gvrp_instance.vehicleFuelConsumptionRate <= compact_model.gvrp_instance.vehicleFuelCapacity && ((costs[f] + distance + costs[r])/compact_model.gvrp_instance.vehicleAverageSpeed) + f0[f].serviceTime + i.serviceTime + j.serviceTime + f0[r].serviceTime <= compact_model.gvrp_instance.timeLimit) { 
                invalidEdge = false;
                f = sf0;
                break;
              }
            }
      if (invalidEdge) {
        //cout<<"Invalid edge here in "<<i.id<<" "<<j.id<<endl;
        for (k = 0; k < compact_model.gvrp_instance.customers.size(); k++) {
          compact_model.model.add(compact_model.x[k][i.id][j.id] == 0);
          compact_model.model.add(compact_model.x[k][j.id][i.id] == 0);
        }
      }
    }
}
