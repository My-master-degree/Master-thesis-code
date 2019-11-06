#ifndef GVRP_INSTANCE_HPP_
#define GVRP_INSTANCE_HPP_

#include "vertex.hpp"
#include <list>
#include <vector>
#include <iostream>

using namespace std;

namespace models{

  class Gvrp_instance {
    public:
      explicit Gvrp_instance(list<Vertex> afss, list<Vertex> customers, Vertex depot, double vehicleFuelCapacity, vector<vector<double> > distances);
//      friend std::ostream& operator<<(std::ostream &strm, const Gvrp_instance &a);
      list<Vertex> afss;
      list<Vertex> customers;
      Vertex depot;
      double vehicleFuelCapacity;
      vector<vector<double> > distances;
  };

//  std::ostream& operator<<(std::ostream& output, const Gvrp_instance& a);

}
#endif
