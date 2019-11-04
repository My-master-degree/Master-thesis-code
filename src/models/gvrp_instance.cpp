#include "models/gvrp_instance.hpp"
#include <list>
#include "models/vertex.hpp"

using gvrp_instance::Gvrp_instance;

Gvrp_instance::Gvrp_instance(void):
  afss(), customers(), depot() {}
Gvrp_instance::Gvrp_instance(list<vertex::Vertex> _afss, list<vertex::Vertex> _customers, vertex::Vertex _depot) :
  afss(_afss), customers(_customers), depot(_depot) {}

