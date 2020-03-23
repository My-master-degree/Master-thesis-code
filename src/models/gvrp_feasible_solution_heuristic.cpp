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
using namespace utils;

Gvrp_feasible_solution_heuristic::Gvrp_feasible_solution_heuristic (Gvrp_instance& gvrp_instance) : Gvrp_heuristic (gvrp_instance) {
  if (gvrp_instance.distances_enum != METRIC)
    throw string("The heuristic 'Feasible Solution' only works for METRIC instances.");
}

Gvrp_solution Gvrp_feasible_solution_heuristic::run () {
  if (gvrp_instance.distances_enum != METRIC)
    throw string("The heuristic 'Feasible Solution' only works for METRIC instances.");
  //create induced graph
  vector<Vertex> f0 = utils::createF0Set (gvrp_instance);
  size_t sf0 = f0.size(),
         r,
         f;
  //dijkstra
  vector<size_t> pred (sf0);
  vector<double> fuels (sf0);
  vector<double> times (sf0);
  utils::gvrpDijkstra(f0, pred, fuels, times, gvrp_instance);
  //build spanning tree
  double routeFuel,
         fuel,
         cost,
          bestAfssFuel;
  list<list<Vertex> > routes;
  list<Vertex> route;  
  for (const Vertex& customer : gvrp_instance.customers) {
    //arg\ min_{f0[f], f0[r] \in F_0 : c_{fi} + c_{ir} \leqslant \beta} {\pi_f + c_{fi} + \pi_r + c_{ir}}
    pair<int, int> bestAfss = make_pair(-1, -1);
    bestAfssFuel = DBL_MAX;
    for (f = 0; f < sf0; f++) 
      if (f0[f].id == gvrp_instance.depot.id || pred[f] != f)
        for (r = 0; r < sf0; r++) 
          //if the afss f and r are connected 
          if (f0[r].id == gvrp_instance.depot.id || pred[r] != r) {
            cost = gvrp_instance.distances[f0[f].id][customer.id] + gvrp_instance.distances[customer.id][f0[r].id];
            fuel = cost * gvrp_instance.vehicleFuelConsumptionRate;
            if (fuel <= gvrp_instance.vehicleFuelCapacity) {
              routeFuel = fuels[f] + fuel + fuels[r] ;
              if (times[f] + (cost/gvrp_instance.vehicleAverageSpeed) + customer.serviceTime + times[r] <= gvrp_instance.timeLimit && routeFuel < bestAfssFuel) {
                bestAfssFuel = routeFuel; 
                bestAfss = make_pair(f, r);
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
