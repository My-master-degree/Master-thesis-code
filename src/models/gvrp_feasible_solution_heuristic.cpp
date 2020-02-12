#include "models/vertex.hpp"
#include "models/gvrp_feasible_solution_heuristic.hpp"
#include "models/gvrp_instance.hpp"
#include "models/gvrp_solution.hpp"
#include "models/distances_enum.hpp"
#include "utils/util.hpp"

#include <float.h>
#include <iostream>
#include <list>
#include <vector>
#include <queue>

using namespace models;

Gvrp_feasible_solution_heuristic::Gvrp_feasible_solution_heuristic (Gvrp_instance& gvrp_instance) : Gvrp_heuristic (gvrp_instance) {
  if (gvrp_instance.distances_enum != METRIC)
    throw string("The heuristic 'Feasible Solution' only works for METRIC instances.");
}

Gvrp_solution Gvrp_feasible_solution_heuristic::run () {
  if (gvrp_instance.distances_enum != METRIC)
    throw string("The heuristic 'Feasible Solution' only works for METRIC instances.");
  //create induced graph
  int size = gvrp_instance.afss.size() + 1;
  vector<Vertex> f0;
  f0.reserve(size);
  f0.push_back(gvrp_instance.depot);
  for (const Vertex& afs : gvrp_instance.afss)
    f0.push_back(afs);
  //dijkstra
  vector<int> pred (size);
  vector<double> costs (size);
  utils::gvrpDijkstra(f0, pred, costs, gvrp_instance);
  //build spanning tree
  int r, f;
  double fuel,
          bestAfssFuel;
  Vertex * v_f, 
          * v_r;
  list<list<Vertex> > routes;
  list<Vertex> route;  
  for (const Vertex& customer : gvrp_instance.customers) {
    //arg\ min_{v_f, v_r \in F_0 : c_{fi} + c_{ir} \leqslant \beta} {\pi_f + c_{fi} + \pi_r + c_{ir}}
    pair<int, int> bestAfss = make_pair(-1, -1);
    bestAfssFuel = DBL_MAX;
    for (f = 0; f < size; f++) {
      v_f = &f0[f];
      for (r = 0; r < size; r++) {
        v_r = &f0[r];
        //if the afss f and r are connected 
        if ((v_f->id == gvrp_instance.depot.id || pred[f] != f) && (v_r->id == gvrp_instance.depot.id || pred[r] != r)) {
          fuel = (gvrp_instance.distances[v_f->id][customer.id] + gvrp_instance.distances[customer.id][v_r->id]) * gvrp_instance.vehicleFuelConsumptionRate;
          if (fuel <= gvrp_instance.vehicleFuelCapacity && (costs[f] + costs[r]) * gvrp_instance.vehicleFuelConsumptionRate + fuel < bestAfssFuel) {
            bestAfssFuel = (costs[f] + costs[r]) * gvrp_instance.vehicleFuelConsumptionRate + fuel; 
            bestAfss = make_pair(f, r);
          }
        }
      }    
    }
    //get route
    if (bestAfss.first >= 0) {
      //way
      for (f = bestAfss.first, route.push_back(f0[f]); f != 0; f = pred[f], route.insert(route.begin(), f0[f]));
      route.push_back(customer);
      //way back
      for (r = bestAfss.second, route.push_back(f0[r]); r != 0; r = pred[r], route.push_back(f0[r]));
      routes.push_back(route);
      route = list<Vertex> ();
    }
  }
  return Gvrp_solution(routes, gvrp_instance);
}
