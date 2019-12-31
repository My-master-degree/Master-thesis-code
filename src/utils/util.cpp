#include "models/gvrp_instance.hpp"
#include "models/distances_enum.hpp"

#include "utils/util.hpp"
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <sstream>
#include <string>
#include <cmath>

using namespace models;

Gvrp_instance utils::erdogan_instance_reader(string file_path){

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
    afss.push_back(Vertex(id++, stod(x, NULL), stod(y, NULL)));
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
    customers.push_back(Vertex(id++, stod(x, NULL), stod(y, NULL)));
    getline(inFile, line);
    if (line.empty())
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
      //distances[i][j] = hypot(vertexes[i].x - vertexes[j].x, vertexes[i].y - vertexes[j].y);
      double dLat = (vertexes[j].y - vertexes[i].y) * M_PI / 180; 
      double dLon = (vertexes[j].x - vertexes[i].x) * M_PI / 180; 
    // apply formulae 
      double a = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(vertexes[i].y * M_PI / 180.0) * cos(vertexes[i].y * M_PI / 180.0); 
      double c = 2 * asin(sqrt(a)); 
      distances[i][j] = radiusOfEarth * c; 
    }
  }
  return Gvrp_instance(afss, customers, depot, vehicleFuelCapacity, distances, METRIC, timeLimit, vehicleFuelConsumptionRate, vehicleAverageSpeed);
}
