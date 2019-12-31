#ifndef GVRP_INSTANCE_HPP_
#define GVRP_INSTANCE_HPP_

#include "vertex.hpp"
#include "distances_enum.hpp"

#include <list>
#include <vector>
#include <string>
#include <sstream>

using namespace std;

namespace models{

  class Gvrp_instance {
    public:
      explicit Gvrp_instance(list<Vertex> afss, list<Vertex> customers, Vertex depot, double vehicleFuelCapacity, vector<vector<double> > distances, Distances_enum distances_enum);
      explicit Gvrp_instance(list<Vertex> afss, list<Vertex> customers, Vertex depot, double vehicleFuelCapacity, vector<vector<double> > distances, Distances_enum distances_enum, double timeLimit, double vehicleFuelConsumptionRate, double vehicleAverageSpeed);
      friend ostream& operator<<(ostream& strm, const Gvrp_instance& gvrp_instance){
        stringstream output;
        output<<"Depot:"<<endl<<"\t"<< gvrp_instance.depot<<endl<<"AFSs: ";
        for (auto afs: gvrp_instance.afss)
          output<<endl<<"\t"<<afs;
        output<<endl<<"Customers:";
        for (auto customer: gvrp_instance.customers)
          output<<endl<<"\t"<<customer;
        return strm << output.str();
      };
      list<Vertex> afss;
      list<Vertex> customers;
      Vertex depot;
      double vehicleFuelCapacity;
      double timeLimit;
      double vehicleFuelConsumptionRate;
      double vehicleAverageSpeed;
      vector<vector<double> > distances;
      Distances_enum distances_enum;
  };

}
#endif
