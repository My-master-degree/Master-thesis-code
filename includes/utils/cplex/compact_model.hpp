#ifndef COMPACT_MODEL_HPP_
#define COMPACT_MODEL_HPP_

#include <list>
#include <models/gvrp_instance.hpp>
#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models;

namespace utils {
  namespace cplex {
    class Compact_model {
      public:
        explicit Compact_model(Gvrp_instance gvrp_instance); 
        list<list<Vertex> > run();
        Gvrp_instance gvrp_instance;
    };

  } 
}
#endif
