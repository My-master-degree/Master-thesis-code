#include "models/vertex.hpp"
#include "models/facility_location_instance.hpp"

#include <list>
#include <vector>

using namespace std;
using namespace models;

Facility_location_instance::Facility_location_instance (list<Vertex>& nodes_, vector<vector<double> > distances_) : nodes (nodes_), distances(distances_) {}
