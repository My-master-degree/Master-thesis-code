#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/gvrp_models/gvrp_afs_tree.hpp"

#include <string> 
#include <float.h>
#include <queue>

using namespace models;
using namespace models::gvrp_models;
using namespace std;

Gvrp_afs_tree::Gvrp_afs_tree(const Gvrp_instance& gvrp_instance) {
  if (gvrp_instance.distances_enum != METRIC)
    throw string("The algorithm 'Gvrp AFS Tree' only works for METRIC instances.");
  //create F0 Set
  size_t size = gvrp_instance.afss.size() + 1;
  f0.reserve(size);
  f0.push_back(&gvrp_instance.depot);
  for (const Vertex& afs : gvrp_instance.afss)
    f0.push_back(&afs);
  //dijkstra
  //setup
  size_t curr,
         f;
  double cost, 
         time, 
         fuel;
  queue<int> q;
  //reserve spaces
  times.reserve(size);
  pred.reserve(size);
  fuels.reserve(size);
  for (f = 0; f < size; f++) 
    pred[f] = f;
  fuels.assign(size, DBL_MAX);
  times.assign(size, DBL_MAX);
  q.push(0);
  fuels[gvrp_instance.depot.id] = 0.0;
  times[gvrp_instance.depot.id] = 0.0;
  while (!q.empty()) {
    curr = q.front();
    q.pop();
    for (f = 0; f < size; f++) {
      cost = gvrp_instance.distances[f0[curr]->id][f0[f]->id];
      fuel = cost * gvrp_instance.vehicleFuelConsumptionRate;
      time = (cost / gvrp_instance.vehicleAverageSpeed) + f0[f]->serviceTime; 
      if (fuel <= gvrp_instance.vehicleFuelCapacity && fuels[curr] + fuel < fuels[f] && times[curr] + time < times[f]) {
        fuels[f] = fuels[curr] + fuel;
        times[f] = times[curr] + time;
        pred[f] = curr;
        q.push(f);
      }
    }
  }
}
