#ifndef GVRP_INSTANCE_HPP_
#define GVRP_INSTANCE_HPP_

#include "vertex.hpp"
#include <list>

using namespace std;

namespace gvrp_instance {
  class Gvrp_instance {
    public:
      explicit Gvrp_instance(list<vertex::Vertex> afss, list<vertex::Vertex> customers, vertex::Vertex depot);
      Gvrp_instance();
      list<vertex::Vertex> afss;
      list<vertex::Vertex> customers;
      vertex::Vertex depot;
  };

} 
#endif
