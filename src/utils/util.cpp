#include "models/vertex.hpp"
#include "models/dsu.hpp"
#include "models/vrp_instance.hpp"
#include "models/distances_enum.hpp"
#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/gvrp_models/gvrp_solution.hpp"
#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp"
#include "models/gvrp_models/gvrp_feasible_solution_heuristic.hpp"
#include "utils/util.hpp"

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
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex::cubic_model;

Gvrp_instance utils::erdogan_instance_reader(const string file_path){
  double time_customer = 30,
          time_afss = 15;
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
  int maxRoutes;
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
  timeLimit = stod(buff.substr(1), NULL) * 60;
  //get vehicle average speed 
  getline(inFile, line);
  ss.clear();
  ss.str(line);  
  while (!ss.eof())
    ss>>buff;  
  vehicleAverageSpeed = stod(buff.substr(1), NULL);
  //get number of routes 
  /*
  getline(inFile, line);
  ss.clear();
  ss.str(line);  
  while (!ss.eof())
    ss>>buff;  
  maxRoutes = stoi(buff.substr(1), NULL);
  */
  inFile.close();
  //save data
  customers_size = customers.size();
  maxRoutes = customers_size;
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
//  double radiusOfEarth = 6371.0; // miles, 6371km; 
  for (int i = 0; i < total_size; i++){
    distances[i] = vector<double> (total_size);
    for (int j = 0; j < total_size; j++){
//      double PI = 4.0*atan(1.0);
      double dLat1 = vertexes[i].y * (M_PI/180.0);
      double dLat2 = vertexes[j].y * (M_PI/180.0);
      double dLat = dLat1 - dLat2; 
      double dLon = (vertexes[i].x - vertexes[j].x) * (M_PI/180.0); 
      double a = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(dLat1) * cos(dLat2); 
//      double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
      double c = 2 * asin(sqrt(a));
      distances[vertexes[i].id][vertexes[j].id] = radiusOfEarth * c;
      //      distances[vertexes[i].id][vertexes[j].id] = sqrt(pow(vertexes[i].x - vertexes[j].x, 2) + pow(vertexes[i].y - vertexes[j].y, 2)); 
    }
  }
  timeLimit = 645;
  return Gvrp_instance(afss, customers, depot, vehicleFuelCapacity, distances, METRIC, maxRoutes, timeLimit, vehicleFuelConsumptionRate, vehicleAverageSpeed);
}

double utils::calculateGvrpInstanceLambda (const Gvrp_instance& gvrp_instance) {
  if (gvrp_instance.afss.size() > 0){
    //calculate lambda
    double lambda = DBL_MAX;
    //min_{v_f \in F} c_{f0} . C
    for (const Vertex& f : gvrp_instance.afss)
      lambda = min(gvrp_instance.distances[f.id][0], lambda);
    return lambda * gvrp_instance.vehicleFuelConsumptionRate;
  }
  return 0.0;  
}

double utils::calculateGvrpInstancePsi (const Gvrp_instance& gvrp_instance) {
  if (gvrp_instance.afss.size() > 1){
    //calculate psi 
    double psi = DBL_MAX;
    //min_{(f, r) \in F : r \neq f} c_{fr} . C
    for (const Vertex& f : gvrp_instance.afss)
      for (const Vertex& r : gvrp_instance.afss)
        if (f.id != r.id)
          psi = min(gvrp_instance.distances[f.id][r.id], psi);
    return psi * gvrp_instance.vehicleFuelConsumptionRate;
  }
  return 0.0;  
}

double utils::calculateVrpInstanceMST (const Vrp_instance& vrp_instance) {
  //build c0 
  const size_t sc0 = vrp_instance.customers.size() + 1;
  auto comp = [](const tuple<int, int, double> & edge1, const tuple<int, int, double> & edge2)
  {
    return get<2>(edge1) > get<2>(edge2);
  };
  priority_queue <tuple<int, int, double>, vector<tuple<int, int, double>>, decltype(comp)> pq (comp); 
  auto end = vrp_instance.customers.end();
  int i = 1;
  for (auto customer_i = vrp_instance.customers.begin(); customer_i != end; ++customer_i, ++i) {
    pq.push (make_tuple(0, i, vrp_instance.distances[vrp_instance.depot.id][customer_i->id]));
    int j = i + 1;
    auto customer_j = customer_i;
    for (++customer_j; customer_j != end; ++customer_j, ++j) 
      pq.push (make_tuple(j, i, vrp_instance.distances[customer_i->id][customer_j->id]));
  }
  DSU dsu(sc0);
  double cost = 0;
  while (!pq.empty()) {
    tuple<int, int, double> item = pq.top();
    pq.pop();
    if (dsu.findSet(get<0>(item)) != dsu.findSet(get<1>(item))) {
      dsu.join(get<0>(item), get<1>(item));
      cost += get<2> (item);
    }
  }
  return cost;
}

double utils::getLongestEdgeUchoaEtAlVrpInstance (const Vrp_instance& vrp_instance) {
  const size_t sall = vrp_instance.customers.size() + 1;
  double maxEdge = 0;
  for (size_t i = 0; i < sall; ++i)
    for (size_t j = 0; j < sall; ++j)
      maxEdge = max (maxEdge, vrp_instance.distances[i][j]);
  return maxEdge;
}

double utils::calculateCustomerMinRequiredFuel (const Gvrp_instance& gvrp_instance, const Gvrp_afs_tree& gvrp_afs_tree, const Vertex& customer) {
  //min_{v_f \in F : s_i + t_{if} + t_{T[f]}} c_{fi} = minAfsFuel 
  double minAfsFuel = DBL_MAX;
  for (size_t f = 0; f < gvrp_afs_tree.f0.size(); f++) 
    if (customer.serviceTime + (gvrp_instance.distances[customer.id][gvrp_afs_tree.f0[f]->id]/gvrp_instance.vehicleAverageSpeed) + gvrp_afs_tree.times[f] < gvrp_instance.timeLimit && gvrp_instance.distances[customer.id][gvrp_afs_tree.f0[f]->id] < minAfsFuel) 
      minAfsFuel = gvrp_instance.distances[customer.id][gvrp_afs_tree.f0[f]->id];    
  return minAfsFuel * gvrp_instance.vehicleFuelConsumptionRate;
}

double utils::calculateCustomerMaxRequiredFuel (const Gvrp_instance& gvrp_instance, const Gvrp_afs_tree& gvrp_afs_tree, const Vertex& customer) {
  return gvrp_instance.vehicleFuelCapacity - utils::calculateCustomerMinRequiredFuel(gvrp_instance, gvrp_afs_tree, customer);
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

vector<vector<int>> utils::read_uchoa_vrp_solution (const string& file_path) {
  ifstream inFile;
  //read file
  inFile.open(file_path);
  if (!inFile)
    throw string("Unable to open file ") + string(file_path);
  //setup
  int nRoutes, 
      nVertexes;
  string buff, 
         line;
  stringstream ss;
  //ignore first line
  getline(inFile, line);
  //get # of routes
  getline(inFile, line);
  ss.str(line);
  ss>>buff;
  ss.clear();
  nRoutes = stoi(buff, NULL);
  vector<vector<int>> routes (nRoutes);
  //ignore two lines
  getline(inFile, line);
  getline(inFile, line);
  //get routes
  for (size_t j = 0; j < nRoutes; ++j) {
    getline(inFile, line);
    ss.str(line);
    ss>>buff;
    ss>>buff;
    ss>>buff;
    ss>>buff;
    ss>>buff;
    ss>>buff;
    nVertexes = stoi(buff, NULL);
    vector<int> route (nVertexes);
    for (int i = 0; i < nVertexes; ++i) {
      ss>>buff;
      route[i] = stoi(buff, NULL);
    }
    routes[j] = route;
    ss.clear();
  }
  inFile.close();
  return routes;
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
  ss>>buff;
  ss>>buff;
  ss>>buff;
  nVertexes = stoi(buff, NULL);
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
  int i = 0;
  for (Vertex a : vertexes) {
    distances[i] = vector<double> (nVertexes);
    for (Vertex b : vertexes) 
      distances[a.id][b.id] = floor(sqrt (pow(a.x - b.x, 2) + pow(a.y - b.y, 2)) + 0.5);
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

list<pair<int, int>> utils::get_invalid_edges_1 (const Gvrp_instance& gvrp_instance) {
  if (gvrp_instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 1' only applies for metric instances");
  list<pair<int, int>> edges;
  for (size_t i = 0; i < gvrp_instance.distances.size(); i++)
    for (size_t j = 0; j < gvrp_instance.distances.size(); j++)
      if (gvrp_instance.distances[i][j] * gvrp_instance.vehicleFuelConsumptionRate > gvrp_instance.vehicleFuelCapacity || gvrp_instance.distances[i][j] / gvrp_instance.vehicleAverageSpeed > gvrp_instance.timeLimit)
        edges.push_back(make_pair(i, j));
  return edges;
}

list<pair<int, int>> utils::get_invalid_edges_2 (const Gvrp_instance& gvrp_instance) {
  if (gvrp_instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 2' only applies for metric instances");
  list<pair<int, int>> edges;
  Gvrp_feasible_solution_heuristic gvrp_feasible_solution_heuristic (gvrp_instance);
  Gvrp_solution gvrp_solution =  gvrp_feasible_solution_heuristic.run();
  set<int> customers;
  //populate customers set
  for (const Vertex& customer : gvrp_instance.customers)
    customers.insert(customer.id);
  //for each route
  for (const list<Vertex>& route : gvrp_solution.routes) {
    auto second = ++route.begin();
    auto penultimate = --route.rbegin();
    //valid preprocessing
    if (!customers.count(second->id) && !customers.count(penultimate->id)) {
      //get customer
      for (second++; !customers.count(second->id); second++);
      edges.push_back(make_pair(gvrp_instance.depot.id, second->id));
      edges.push_back(make_pair(second->id, gvrp_instance.depot.id));
    }
  }
  return edges;
}

list<pair<int, int>> utils::get_invalid_edges_3 (const Gvrp_instance& gvrp_instance, const Gvrp_afs_tree& gvrp_afs_tree) {
  if (gvrp_instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 3' only applies for metric instances");
  list<pair<int, int>> edges;
  //create induced graph
  const vector<const Vertex *>& f0 = gvrp_afs_tree.f0;
  const vector<size_t>& pred = gvrp_afs_tree.pred;
  const vector<double>& times = gvrp_afs_tree.times;
  size_t sf0 = f0.size(),
          r,
          f;
  //get invalid edges
  bool invalidEdge;
  double distance;
  for (const Vertex& i : gvrp_instance.customers) 
    for (const Vertex& j : gvrp_instance.customers) {
      invalidEdge = true;
      //check if edge (i, j) can be feasible
      for (f = 0; f < sf0; f++) 
        if (f0[f]->id == gvrp_instance.depot.id || pred[f] != f) 
          for (r = 0; r < sf0; r++) 
            //if the afss f and r are connected 
            if (f0[r]->id == gvrp_instance.depot.id || pred[r] != r) {
              distance = gvrp_instance.distances[f0[f]->id][i.id] + gvrp_instance.distances[i.id][j.id] + gvrp_instance.distances[j.id][f0[r]->id];
              if (distance * gvrp_instance.vehicleFuelConsumptionRate <= gvrp_instance.vehicleFuelCapacity && (times[f] + (distance/gvrp_instance.vehicleAverageSpeed) + i.serviceTime + j.serviceTime + times[r] <= gvrp_instance.timeLimit)) { 
                invalidEdge = false;
                f = sf0;
                break;
              }
            }
      if (invalidEdge) {
        edges.push_back(make_pair(i.id, j.id));
        edges.push_back(make_pair(j.id, i.id));
      }
    }
  return edges;
}

list<pair<int, int>> utils::get_invalid_edges_4 (const Gvrp_instance& gvrp_instance, const Gvrp_afs_tree& gvrp_afs_tree) {
  if (gvrp_instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 4' only applies for metric instances");
  list<pair<int, int>> edges;
  //create induced graph
  const vector<const Vertex *>& f0 = gvrp_afs_tree.f0;
  const vector<size_t>& pred = gvrp_afs_tree.pred;
  const vector<double>& times = gvrp_afs_tree.times;
  size_t sf0 = f0.size(),
          r,
          f;
  //get invalid edges
  bool invalidEdge;
  double distance;
  for (const Vertex& i : gvrp_instance.customers)
    for (f = 0; f < sf0; f++) {
      invalidEdge = true;
      if (f0[f]->id == gvrp_instance.depot.id || pred[f] != f) 
        for (r = 0; r < sf0; r++) 
          //if the afss f and r are connected 
          if (f0[r]->id == gvrp_instance.depot.id || pred[r] != r) {
            distance = gvrp_instance.distances[f0[f]->id][i.id] + gvrp_instance.distances[i.id][f0[r]->id];
            if (distance * gvrp_instance.vehicleFuelConsumptionRate <= gvrp_instance.vehicleFuelCapacity && times[f] + (distance/gvrp_instance.vehicleAverageSpeed) + i.serviceTime + times[r] <= gvrp_instance.timeLimit) { 
              invalidEdge = false;
              f = sf0;
              break;
            }
          }
      if (invalidEdge) {
        edges.push_back(make_pair(i.id, f0[f]->id));
        edges.push_back(make_pair(f0[f]->id, i.id));
      }
    }
  return edges;
}

