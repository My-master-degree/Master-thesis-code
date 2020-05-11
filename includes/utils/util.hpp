#ifndef UTIL_HPP_
#define UTIL_HPP_

#include "models/vertex.hpp"
#include "models/vrp_instance.hpp"
#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/gvrp_models/gvrp_afs_tree.hpp"
#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp"

#include <string>
#include <list>
#include <map>
#include <vector>

using namespace models;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex::cubic_model;
using namespace std;

namespace utils {
  Vrp_instance read_uchoa_vrp_instance (const string &file_path);    
  Gvrp_instance erdogan_instance_reader(const string file_path);
  list<string> listFilesFromDir(string path);
  double calculateGvrpInstanceLambda (const Gvrp_instance& gvrp_instance);
  double calculateGvrpInstancePsi (const Gvrp_instance& gvrp_instance);
  double calculateVrpInstanceMST (const Vrp_instance& vrp_instance);
  double calculateCustomerMinRequiredFuel (const Gvrp_instance& gvrp_instance, const Gvrp_afs_tree& gvrp_afs_tree, const Vertex& customer);
  double calculateCustomerMaxRequiredFuel (const Gvrp_instance& gvrp_instance, const Gvrp_afs_tree& gvrp_afs_tree, const Vertex& customer);
  list<list<Vertex> > getGvrpConnectedComponents (const Gvrp_instance& gvrp_instance);
  list<pair<int, int>> get_invalid_edges_1 (const Gvrp_instance& gvrp_instance);
  list<pair<int, int>> get_invalid_edges_2 (const Gvrp_instance& gvrp_instance);
  list<pair<int, int>> get_invalid_edges_3 (const Gvrp_instance& gvrp_instance, const Gvrp_afs_tree& gvrp_afs_tree);
  list<pair<int, int>> get_invalid_edges_4 (const Gvrp_instance& gvrp_instance, const Gvrp_afs_tree& gvrp_afs_tree);
} 
#endif
