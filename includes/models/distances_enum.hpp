#ifndef DISTANCES_ENUM_HPP_
#define DISTANCES_ENUM_HPP_

#include <list>
#include <models/gvrp_instance.hpp>
#include <ilcplex/ilocplex.h>

using namespace std;

namespace models {
  enum Distances_enum {
    SYMMETRIC, METRIC, NONE
  };
}
#endif
