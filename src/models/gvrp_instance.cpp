#include "models/distances_enum.hpp"
#include "models/vertex.hpp"
#include "models/gvrp_instance.hpp"

#include <vector>
#include <list>
#include <fstream>

using namespace models;
using namespace std;

Gvrp_instance::Gvrp_instance(list<Vertex> afss_, list<Vertex> customers_, Vertex depot_, double vehicleFuelCapacity_, vector<vector<double> > distances_, Distances_enum distances_enum_, int nRoutes) : Vrp_instance (customers_, depot_, distances_, distances_enum_, nRoutes), afss(afss_), vehicleFuelCapacity(vehicleFuelCapacity_) {
  timeLimit = 1000000000;
  vehicleFuelConsumptionRate = 1.0;
  vehicleAverageSpeed = 1.0;
}

Gvrp_instance::Gvrp_instance(list<Vertex> _afss, list<Vertex> _customers, Vertex _depot, double _vehicleFuelCapacity, vector<vector<double> > _distances, Distances_enum _distances_enum, int nRoutes, double _timeLimit, double _vehicleFuelConsumptionRate, double _vehicleAverageSpeed) : Gvrp_instance(_afss, _customers, _depot, _vehicleFuelCapacity, _distances, _distances_enum, nRoutes) {
  timeLimit = _timeLimit;
  vehicleFuelConsumptionRate = _vehicleFuelConsumptionRate;
  vehicleAverageSpeed = _vehicleAverageSpeed;
}

void Gvrp_instance::write_in_csv(const string& file_path){
  ofstream instanceFile;
  double fuel, 
         time;
  instanceFile.open (file_path);
  //params
  instanceFile<<"Vehicle:"<<endl<<"Average speed:;"<<this->vehicleAverageSpeed<<endl<<"Time limit:;"<<this->timeLimit<<endl<<"Fuel comsumption rate:;"<<this->vehicleFuelConsumptionRate<<endl<<"Fuel Capacity:;"<<this->vehicleFuelCapacity<<endl;
  //depot
  instanceFile <<"Depot:"<<endl;
  instanceFile <<"ID;X;Y;Service Time"<<endl;
  instanceFile<<depot.id<<";"<<depot.x<<";"<<depot.y<<";"<<depot.serviceTime<<endl;
  //customers
  instanceFile <<"Customers:"<<endl;
  instanceFile <<"ID;X;Y;Service Time"<<endl;
  for (const Vertex& customer : this->customers)
    instanceFile<<customer.id<<";"<<customer.x<<";"<<customer.y<<";"<<customer.serviceTime<<endl;
  //afss
  instanceFile <<"AFSs:"<<endl;
  instanceFile <<"ID;X;Y;Service Time"<<endl;
  for (const Vertex& afs: this->afss)
    instanceFile<<afs.id<<";"<<afs.x<<";"<<afs.y<<";"<<afs.serviceTime<<endl;
  //distances matrix
  instanceFile <<"Distance matrix:"<<endl;
  for (int i = 0; i < int(this->distances.size()); i++)
    instanceFile <<";"<<i;
  instanceFile <<endl;
  for (int i = 0; i < int(this->distances.size()); i++) {
    instanceFile <<i;
    for (int j = 0; j < int(this->distances.size()); j++) 
      instanceFile <<";"<< int(this->distances[i][j]) << "."<<int(this->distances[i][j] * 100)%100;
    instanceFile <<endl;
  }
  //fuel matrix
  instanceFile <<"Fuel matrix:"<<endl;
  for (int i = 0; i < int(this->distances.size()); i++)
    instanceFile <<";"<<i;
  instanceFile <<endl;
  for (int i = 0; i < int(this->distances.size()); i++) {
    instanceFile <<i;
    for (int j = 0; j < int(this->distances.size()); j++) {
      fuel = this->distances[i][j] * this->vehicleFuelConsumptionRate;
      instanceFile <<";"<< int(fuel) << "."<<int(fuel * 100)%100;
    }
    instanceFile <<endl;
  }
  //time matrix
  instanceFile <<"Time matrix:"<<endl;
  for (int i = 0; i < int(this->distances.size()); i++)
    instanceFile <<";"<<i;
  instanceFile <<endl;
  for (int i = 0; i < int(this->distances.size()); i++) {
    instanceFile <<i;
    for (int j = 0; j < int(this->distances.size()); j++) {
      time = this->distances[i][j] / this->vehicleAverageSpeed;
      instanceFile <<";"<< int(time) << "."<<int(time * 100)%100;
    }
    instanceFile <<endl;
  }
  instanceFile.close(); 
}
