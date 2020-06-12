#ifndef UTIL_HPP_
#define UTIL_HPP_

#include "models/vertex.hpp"
#include "models/vrp_instance.hpp"
#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/gvrp_models/gvrp_solution.hpp"
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
  pair<vector<double>, vector<double>> calculateClosestsVRPCustomers (const Vrp_instance& vrp_instance, const vector<const Vertex *>& vertices);
  int calculateGVRP_BPP_NRoutesLB(const Gvrp_instance& gvrp_instance, const vector<const Vertex *>& vertices, const vector<double>& closest, const vector<double>& secondClosest, unsigned int execution_time_limit);
  double calculate_TSP_LB (const vector<const Vertex *>& vertices, const vector<double>& closest, const vector<double>& closestSecond);
  double calculateVRPSolutionCost (const vector<vector<int>>& routes, const Vrp_instance& vrp_instance);
  double calculateLargestRouteCost (const vector<vector<int>>& routes, const Vrp_instance& vrp_instance); 
  double calculateRouteAverageCost (const vector<vector<int>>& routes, const Vrp_instance& vrp_instance);
  Gvrp_instance matheus_instance_reader(const string& file_path);
  Vrp_instance read_uchoa_vrp_instance (const string &file_path);    
  vector<vector<int>> read_uchoa_vrp_solution (const string &file_path);    
  Gvrp_instance erdogan_instance_reader(const string file_path);
  list<string> listFilesFromDir(string path);
  Gvrp_solution read_gvrp_solution (const string& file_path, const Gvrp_instance& gvrp_instance);
  double calculateGvrpInstanceLambda (const Gvrp_instance& gvrp_instance);
  double calculateGvrpInstancePsi (const Gvrp_instance& gvrp_instance);
  double calculateVrpMST (const Vrp_instance& vrp_instance, const vector<const Vertex *>& vertices);
  double getLongestEdgeUchoaEtAlVrpInstance (const Vrp_instance& vrp_instance);
  double calculateCustomerMinRequiredFuel (const Gvrp_instance& gvrp_instance, const Gvrp_afs_tree& gvrp_afs_tree, const Vertex& customer);
  double calculateCustomerMaxRequiredFuel (const Gvrp_instance& gvrp_instance, const Gvrp_afs_tree& gvrp_afs_tree, const Vertex& customer);
  list<list<Vertex> > getGvrpConnectedComponents (const Gvrp_instance& gvrp_instance);
  list<pair<int, int>> get_invalid_edges_1 (const Gvrp_instance& gvrp_instance);
  list<pair<int, int>> get_invalid_edges_2 (const Gvrp_instance& gvrp_instance);
  list<pair<int, int>> get_invalid_edges_3 (const Gvrp_instance& gvrp_instance, const Gvrp_afs_tree& gvrp_afs_tree);
  list<pair<int, int>> get_invalid_edges_4 (const Gvrp_instance& gvrp_instance, const Gvrp_afs_tree& gvrp_afs_tree);
} 
#endif
