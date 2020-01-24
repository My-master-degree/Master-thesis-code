#ifndef GVRP_HEURISTIC_HPP_
#define GVRP_HEURISTIC_HPP_

#include "models/gvrp_instance.hpp"
#include "models/gvrp_solution.hpp"

namespace models {
  class Gvrp_heuristic {
    public:
      Gvrp_instance& gvrp_instance;
      explicit Gvrp_heuristic (Gvrp_instance& gvrp_instance);
      virtual Gvrp_solution run() = 0;
  };
}

#endif
