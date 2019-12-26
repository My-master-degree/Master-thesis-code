#include "models/gvrp_instance.hpp"
#include "models/distances_enum.hpp"
#include "models/vertex.hpp"

#include <vector>
#include <list>

using namespace models;
using namespace std;

Gvrp_instance::Gvrp_instance(list<Vertex> _afss, list<Vertex> _customers, Vertex _depot, double _vehicleFuelCapacity, vector<vector<double> > _distances, Distances_enum _distances_enum) :
  afss(_afss), customers(_customers), depot(_depot), vehicleFuelCapacity(_vehicleFuelCapacity), distances(_distances), distances_enum (_distances_enum) {
}


