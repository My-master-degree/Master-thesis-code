#ifndef UTIL_HPP_
#define UTIL_HPP_

#include "models/vertex.hpp"
#include "models/vrp_instance.hpp"
#include "models/gvrp_instance.hpp"
#include "utils/cplex/compact_model.hpp"

#include <string>
#include <list>
#include <map>

using namespace models;
using namespace utils::cplex;

namespace utils {
  Vrp_instance read_uchoa_vrp_instance (const string &file_path);    
  Gvrp_instance erdogan_instance_reader(string file_path);
  list<string> listFilesFromDir(string path);
  void generate_new_gvrp_instances (); 
  double calculateGvrpInstanceLambdaFactor (const Gvrp_instance& gvrp_instance);
  list<list<Vertex> > getGvrpConnectedComponents (const Gvrp_instance& gvrp_instance);
  map<int, double> calculateCustomersEnergyUB (Compact_model& gvrp_instance);
} 
#endif
