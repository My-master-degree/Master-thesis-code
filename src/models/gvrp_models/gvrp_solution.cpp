#include "models/gvrp_models/gvrp_solution.hpp"
#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/vertex.hpp"

#include <list>
#include <iterator>
#include <iostream>
#include <fstream>
#include <string>

using namespace models;
using namespace models::gvrp_models;
using namespace std;

Gvrp_solution::Gvrp_solution(list<list<Vertex> > _routes, Gvrp_instance _gvrp_instance) :
  routes(_routes), gvrp_instance(_gvrp_instance) {
}

void Gvrp_solution::write_in_file(const string& file_path){
  ofstream solutionFile;
  solutionFile.open (file_path);
  solutionFile <<*this;
  solutionFile.close(); 
}

double Gvrp_solution::calculateCost () {
  double cost = 0;
  for (list<Vertex>& route : routes) 
    for (list<Vertex>::iterator c = next(route.begin()); c != route.end(); ++c)
      cost += gvrp_instance.distances[prev(c)->id][c->id];
  return cost;
}
