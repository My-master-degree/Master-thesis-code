#include "models/vertex.hpp"
#include "models/vrp_instance.hpp"
#include "models/gvrp_instance.hpp"
#include "models/distances_enum.hpp"
#include "utils/util.hpp"
#include "models/cubic_model/cubic_model.hpp"

#include <float.h>
#include <climits>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <sstream>
#include <string>
#include <string.h>
#include <cmath>
#include <map>
#include <set>
#include <queue>
#include <dirent.h>
#include <sys/types.h>
#include <list>
#include "SampleConfig.h"

using namespace models;
using namespace models::cubic_model;

Gvrp_instance utils::erdogan_instance_reader(string file_path){
  double time_customer = 0.5,
          time_afss = 0.25;
  int id = 0,
      customers_size,
      afss_size,
      total_size;
  string buff, 
         type, 
         x, 
         y,
         line;
  ifstream inFile;
  list<Vertex> afss, 
    customers;
  stringstream ss;
  Vertex depot;
  int nRoutes;
  double vehicleFuelCapacity,
         vehicleFuelConsumptionRate,
         timeLimit,
         vehicleAverageSpeed;
  //read file
  inFile.open(file_path);
  if (!inFile)
    throw string("Unable to open file ") + string(file_path);
  //ignore header
  getline(inFile, line);
  //get depot
  getline(inFile, line);
  ss.str(line);
  //ignore id
  ss>>buff;
  ss>>type;
  ss>>x;
  ss>>y;
  depot = Vertex(id++, stod(x, NULL), stod(y, NULL));
  //get afs's
  getline(inFile, line);
  ss.clear();
  ss.str(line);
  //ignore id
  ss>>buff;
  //get type
  ss>>type;
  ss>>x;
  ss>>y;
  while (type == "f"){         
    afss.push_back(Vertex(id++, stod(x, NULL), stod(y, NULL), time_afss));
    getline(inFile, line);
    ss.clear();
    ss.str(line);
    //ignore id
    ss>>buff;
    //get type
    ss>>type;
    ss>>x;
    ss>>y;
  }
  //get customers
  while (type == "c"){         
    customers.push_back(Vertex(id++, stod(x, NULL), stod(y, NULL), time_customer));
    getline(inFile, line);
    if (line.empty() || line == "\n"|| line == "\r")
      break;
    ss.clear();
    ss.str(line);
    //ignore id
    ss>>buff;
    //get type
    ss>>type;
    ss>>x;
    ss>>y;
  }
  //get beta
  getline(inFile, line);
  ss.clear();
  ss.str(line);  
  while (!ss.eof())
    ss>>buff;  
  vehicleFuelCapacity = stod(buff.substr(1), NULL);
  //get vehicle fuel consumption rate 
  getline(inFile, line);
  ss.clear();
  ss.str(line);  
  while (!ss.eof())
    ss>>buff;  
  vehicleFuelConsumptionRate = stod(buff.substr(1), NULL);
  //get time limit 
  getline(inFile, line);
  ss.clear();
  ss.str(line);  
  while (!ss.eof())
    ss>>buff;  
  timeLimit = stod(buff.substr(1), NULL);
  //get vehicle average speed 
  getline(inFile, line);
  ss.clear();
  ss.str(line);  
  while (!ss.eof())
    ss>>buff;  
  vehicleAverageSpeed = stod(buff.substr(1), NULL);
  //get number of routes 
  getline(inFile, line);
  ss.clear();
  ss.str(line);  
  while (!ss.eof())
    ss>>buff;  
  nRoutes = stoi(buff.substr(1), NULL);
  inFile.close();
  //save data
  customers_size = customers.size();
  afss_size = afss.size();
  total_size = customers_size + afss_size + 1;
  //calculate distances
  vector<Vertex> vertexes (total_size);
  int j = 0; 
  vertexes[j++] = depot;
  for (list<Vertex>::iterator i = afss.begin(); i != afss.end(); i++)
    vertexes[j++] = *i;
  for (list<Vertex>::iterator i = customers.begin(); i != customers.end(); i++)
    vertexes[j++] = *i;
  vector<vector<double> > distances(total_size);
  double radiusOfEarth = 4182.44949; // miles, 6371km; 
//  double radiusOfEarth = 6373.0; // miles, 6371km; 
  for (int i = 0; i < total_size; i++){
    distances[i] = vector<double> (total_size);
    for (int j = 0; j < total_size; j++){
//      double PI = 4.0*atan(1.0);
      double dLat1 = vertexes[i].y * (M_PI/180);
      double dLat2 = vertexes[j].y * (M_PI/180);
      double dLon1 = vertexes[i].x * (M_PI/180);
      double dLon2 = vertexes[j].x * (M_PI/180);

      double dLat = dLat1 - dLat2; 
      double dLon = dLon1 - dLon2; 
      double a = pow(sin(dLat / 2.0), 2.0) + pow(sin(dLon / 2.0), 2.0) * cos(dLat1) * cos(dLat2); 
      double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
      distances[vertexes[i].id][vertexes[j].id] = radiusOfEarth * c;
      //      distances[vertexes[i].id][vertexes[j].id] = sqrt(pow(vertexes[i].x - vertexes[j].x, 2) + pow(vertexes[i].y - vertexes[j].y, 2)); 
    }
  }
  return Gvrp_instance(afss, customers, depot, vehicleFuelCapacity, distances, METRIC, nRoutes, timeLimit, vehicleFuelConsumptionRate, vehicleAverageSpeed);
}

double utils::calculateGvrpInstanceLambdaFactor (const Gvrp_instance& gvrp_instance) {
  if (gvrp_instance.afss.size() > 0){
    //calculate lambda
    double lambda = DBL_MAX;
    //F_0
    set<int> afssAndDepot;
    for (Vertex afs : gvrp_instance.afss)
      afssAndDepot.insert(afs.id);
    afssAndDepot.insert(gvrp_instance.depot.id);
    //min_{(f, r) \in F_0 x F_0: r \neq f} c_{fr} . C
    for (int f : afssAndDepot)
      for (int r : afssAndDepot)
        if (f != r)
          lambda = min(gvrp_instance.distances[f][r] * gvrp_instance.vehicleFuelConsumptionRate, lambda);
    //min_{(i, 0) \in C x \{v_0\}} c_{i0} . C
    for (const Vertex& i : gvrp_instance.customers)
      lambda = min(gvrp_instance.distances[i.id][0] * gvrp_instance.vehicleFuelConsumptionRate, lambda);
    return lambda;
  }
  return 0.0;  
}

list<string> utils::listFilesFromDir(string path) {
  list<string> files;
  struct dirent *entry;
  DIR *dir = opendir(path.c_str());
  if (dir == NULL) 
    throw string("Error in accessing ") + path;
  while ((entry = readdir(dir)) != NULL) 
    if (strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0)
      files.push_back (entry->d_name);
  closedir(dir);
  files.sort();
  return files; 
}

void utils::generate_new_gvrp_instances () {
  //
  string uchoaInstancesDir = PROJECT_INSTANCES_PATH + string("UchoaEtAl/");
  list<string> instances = utils::listFilesFromDir(uchoaInstancesDir);
  for (string instance : instances) {
    Vrp_instance vrp_instance = utils::read_uchoa_vrp_instance (uchoaInstancesDir + instance);

  }
}

Vrp_instance utils::read_uchoa_vrp_instance (const string& file_path) {
  int id = 0,
      nVertexes;
  string buff, 
         x, 
         y,
         line;
  ifstream inFile;
  list<Vertex> vertexes;
  stringstream ss;
  //read file
  inFile.open(file_path);
  if (!inFile)
    throw string("Unable to open file ") + string(file_path);
  //ignore header
  for (int i = 0; i < 4; i++)
    getline(inFile, line);
  //get # vertexes
  ss.str(line);
  //ignore text 
  ss>>buff;
  ss>>buff;
  //get #
  ss>>nVertexes;
  ss.clear();
  //ignore header
  for (int i = 0; i < 3; i++)
    getline(inFile, line);
  //get vertexes 
  for (int i = 0; i < nVertexes; i++) {
    getline(inFile, line);
    ss.str(line);
    //ignore id
    ss>>buff;
    //get axis
    ss>>x;
    ss>>y;
    vertexes.push_back(Vertex(id++, stod(x, NULL), stod(y, NULL)));
  } 
  //calculate distances
  vector<vector<double> > distances(nVertexes);
  int i = 0, j;
  for (Vertex a : vertexes) {
    distances[i] = vector<double> (nVertexes);
    j = 0;
    for (Vertex b : vertexes) {
      distances[a.id][b.id] = distances[a.id][b.id] = sqrt (pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
      j++;
    }
    i++;
  }
  //get depot
  Vertex depot = Vertex(*vertexes.begin());
  //    Vertex depot = Vertex (vertexes.begin()->id, vertexes.begin()->x, vertexes.begin()->y);
  //remove header
  vertexes.erase (vertexes.begin());
  return Vrp_instance (vertexes, depot, distances, METRIC, vertexes.size());
} 

list<list<Vertex> > utils::getGvrpConnectedComponents (const Gvrp_instance& gvrp_instance) {
  //setup
  list<list<Vertex>> components; 
  int nNodes = gvrp_instance.distances.size();
  vector<bool> visited (nNodes, false);
  vector<Vertex> all (nNodes);
  set<int> customers;
  list<Vertex> component;
  queue<int> q;
  double phi;
  int curr,
      j;
  for (const Vertex& customer : gvrp_instance.customers) {
    customers.insert(customer.id);
    all[customer.id] = customer;
  }
  for (const Vertex& afs : gvrp_instance.afss)
    all[afs.id] = afs;
  //bfs
  for (const Vertex& customer : gvrp_instance.customers) {
    if (!visited[customer.id]) {
      q.push(customer.id); 
      visited[customer.id] = true;
      while (!q.empty()) {
        curr = q.front();
        q.pop();
        //get phi
        if (customers.count(curr)) {
          phi = DBL_MAX; 
          for (j = 0; j < int(gvrp_instance.distances.size()); j++) 
            if (curr != j)
              phi = min (phi, gvrp_instance.distances[j][curr]);
          phi = gvrp_instance.vehicleFuelCapacity - phi * gvrp_instance.vehicleFuelConsumptionRate;
        } else 
          phi = gvrp_instance.vehicleFuelCapacity;
        //seve in the list
        component.push_back(all[curr]);
        //get neighboring
        for (j = 0; j < int(gvrp_instance.distances.size()); j++) 
          if (!visited[j] && j != gvrp_instance.depot.id && phi >= gvrp_instance.distances[curr][j] * gvrp_instance.vehicleFuelConsumptionRate) {
            visited[j] = true;
            q.push(j);
          }
      }
      components.push_back(component);
      component = list<Vertex> ();
    }
  }
  return components;
}

map<int, double> utils::calculateCustomersEnergyUB (Cubic_model& cubic_model) {
  map<int, double> customersEnergyUBs;
  for (Vertex customer : cubic_model.gvrp_instance.customers){
    int i = customer.id;
    //min_{(j, i) \in E} c_{ji} = minEdge
    double minEdge = DBL_MAX;
    for (pair <int, Vertex> p : cubic_model.all) {
      int j = p.first;
      if (i != j)
        minEdge = min (minEdge, cubic_model.gvrp_instance.distances[i][j]);
    }
    customersEnergyUBs[i] = cubic_model.gvrp_instance.vehicleFuelCapacity - minEdge * cubic_model.gvrp_instance.vehicleFuelConsumptionRate;
  }
  return customersEnergyUBs;
}

vector<Vertex> utils::createF0Set (Gvrp_instance& gvrp_instance) {
  int size = gvrp_instance.afss.size() + 1;
  vector<Vertex> f0;
  f0.reserve(size);
  f0.push_back(gvrp_instance.depot);
  for (const Vertex& afs : gvrp_instance.afss)
    f0.push_back(afs);
  return f0;
}

void utils::gvrpDijkstra (vector<Vertex>& f0, vector<size_t>& pred, vector<double>& costs, Gvrp_instance& gvrp_instance) {
  if (gvrp_instance.distances_enum != METRIC)
    throw string("The algorithm 'Gvrp Dijkstra' only works for METRIC instances.");
  size_t size = f0.size();
  if (pred.size() < size) {
    stringstream ss;
    ss<<"The pred vector requires at least "<<size<<" of length";
    throw ss.str();
  }
  if (costs.size() < size) {
    stringstream ss;
    ss<<"The costs vector requires at least "<<size<<" of length";
    throw ss.str();
  }
  size_t curr,
      f;
  double cost;
  for (f = 0; f < size; f++) 
    pred[f] = f;
  costs.assign(size, DBL_MAX);
  queue<int> q;
  q.push(0);
  costs[0] = 0.0;
  while (!q.empty()) {
    curr = q.front();
    q.pop();
    for (f = 0; f < size; f++) {
      cost = gvrp_instance.distances[f0[curr].id][f0[f].id];
      if (cost * gvrp_instance.vehicleFuelConsumptionRate <= gvrp_instance.vehicleFuelCapacity && costs[curr] + cost < costs[f]) {
        costs[f] = costs[curr] + cost;
        pred[f] = curr;
        q.push(f);
      }
    }
  }
}
