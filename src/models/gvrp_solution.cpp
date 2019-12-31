#include "models/gvrp_solution.hpp"
#include "models/gvrp_instance.hpp"
#include "models/vertex.hpp"

#include <list>

using namespace models;
using namespace std;

Gvrp_solution::Gvrp_solution(list<list<Vertex> > _routes, Gvrp_instance _gvrp_instance) :
  routes(_routes), gvrp_instance(_gvrp_instance) {
}


