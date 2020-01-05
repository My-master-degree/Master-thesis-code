#include "models/gvrp_instance.hpp"
#include "models/distances_enum.hpp"
#include "utils/util.hpp"

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <sstream>
#include <string>
#include <cmath>
#include <map>
#include <set>
#include <queue>

using namespace models;

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
  double vehicleFuelCapacity,
         vehicleFuelConsumptionRate,
         timeLimit,
         vehicleAverageSpeed;
  //read file
  inFile.open(file_path);
  if (!inFile) {
    cerr << "Unable to open file "<<file_path;
    exit(1);   // call system to stop
  }  
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
  inFile.close();
  //save data
  customers_size = customers.size();
  afss_size = afss.size();
  total_size = customers_size + afss_size + 1;
  //calculate distances
  vector<Vertex> vertexes (total_size);
  int j = 0; 
  vertexes[j++] = depot;
  for (list<Vertex>::iterator i = customers.begin(); i != customers.end(); i++)
    vertexes[j++] = *i;
  for (list<Vertex>::iterator i = afss.begin(); i != afss.end(); i++)
    vertexes[j++] = *i;
  vector<vector<double> > distances(total_size);
  
  double radiusOfEarth = 4182.44949; // miles, 6371km; 
  for (int i = 0; i < total_size; i++){
    distances[i] = vector<double> (total_size);
    for (int j = 0; j < total_size; j++){
//      distances[i][j] = hypot(vertexes[i].x - vertexes[j].x, vertexes[i].y - vertexes[j].y);
//        double dLat = Math.toRadians(lat2-lat1); 
//      double dLon = Math.toRadians(lon2-lon1); 
//      double a = Math.sin(dLat/2) * Math.sin(dLat/2) + Math.cos(Math.toRadians(lat1)) * Math.cos(Math.toRadians(lat2)) * Math.sin(dLon/2) * Math.sin(dLon/2); 
//      double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a)); 
//      double distance = radiusOfEarth * c; 
//      return distance;


      double dLat = (vertexes[i].y - vertexes[j].y) * M_PI / 180; 
     double dLon = (vertexes[i].x - vertexes[j].x) * M_PI / 180; 
   // apply formulae 
     double a = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(vertexes[i].y * M_PI / 180.0) * cos(vertexes[j].y * M_PI / 180.0); 
 //    double c = 2 * asin(sqrt(a)); 
        double c = 2* atan2(sqrt(a), sqrt(1-a));
     distances[i][j] = int(radiusOfEarth * c * 100) / 100.0;
    }
  }
  return Gvrp_instance(afss, customers, depot, vehicleFuelCapacity, distances, METRIC, timeLimit, vehicleFuelConsumptionRate, vehicleAverageSpeed);
}

void utils::remove_infeasible_customers(Gvrp_instance& gvrp_instance){
  if (gvrp_instance.distances_enum == METRIC){
    //setup vars
    map<int, Vertex> afss;
    for (auto afs: gvrp_instance.afss)
      afss[afs.id] = afs;
    set<int> coveredAfss;
    queue<int> q;
    //bfs
    q.push(gvrp_instance.depot.id);
    while (!q.empty()){
      int curr = q.front();
      q.pop();
      coveredAfss.insert(curr);
      for (auto afs: gvrp_instance.afss)
        //if afs is a valid afs
        if (gvrp_instance.distances[curr][afs.id] * gvrp_instance.vehicleFuelConsumptionRate <= gvrp_instance.vehicleFuelCapacity && !coveredAfss.count(afs.id))
          q.push(afs.id);          
    }
    //check feasible customers
    for (auto it = gvrp_instance.customers.begin(); it != gvrp_instance.customers.end();){
      Vertex* customer = &(*it);
//      cout<<"Iterating over customer "<<customer->id<<endl;
      bool feasibleCustomer = false;
      for (auto coveredAfs : coveredAfss)
        if (gvrp_instance.distances[customer->id][coveredAfs] * gvrp_instance.vehicleFuelConsumptionRate <= gvrp_instance.vehicleFuelCapacity / 2.0){
          feasibleCustomer = true;
          break;
        }
      if (gvrp_instance.distances[customer->id][gvrp_instance.depot.id] * gvrp_instance.vehicleFuelConsumptionRate > gvrp_instance.vehicleFuelCapacity / 2.0 && !feasibleCustomer){
        
//        cout<<"Customer "<<customer->id<<" removed"<<endl;
        it = gvrp_instance.customers.erase(it);
      }else
        ++it;
    }
  }else
    throw "It is not possible to remove infeasible customers from a non-metric instance.";
}
