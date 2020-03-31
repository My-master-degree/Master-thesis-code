#ifndef UTIL_HPP_
#define UTIL_HPP_

#include "models/vertex.hpp"
#include "models/vrp_instance.hpp"
#include "models/gvrp_instance.hpp"
#include "models/cubic_model/cubic_model.hpp"

#include <string>
#include <list>
#include <map>
#include <vector>

using namespace models;
using namespace models::cubic_model;
using namespace std;

namespace utils {
  Vrp_instance read_uchoa_vrp_instance (const string &file_path);    
  Gvrp_instance erdogan_instance_reader(string file_path);
  list<string> listFilesFromDir(string path);
  double calculateGvrpInstanceLambdaFactor (const Gvrp_instance& gvrp_instance);
  list<list<Vertex> > getGvrpConnectedComponents (const Gvrp_instance& gvrp_instance);
  map<int, double> calculateCustomersEnergyUB (Cubic_model& gvrp_instance);
  void gvrpDijkstra (vector<Vertex>& f0, vector<size_t>& pred, vector<double>& fuels, vector<double>& times, Gvrp_instance& gvrp_instance);
  vector<Vertex> createF0Set (Gvrp_instance& gvrp_instance);
  list<pair<int, int>> get_invalid_edges_1 (Gvrp_instance& gvrp_instance);
  list<pair<int, int>> get_invalid_edges_2 (Gvrp_instance& gvrp_instance);
  list<pair<int, int>> get_invalid_edges_3 (Gvrp_instance& gvrp_instance);
  list<pair<int, int>> get_invalid_edges_4 (Gvrp_instance& gvrp_instance);
} 
#endif
