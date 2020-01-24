#ifndef GVRP_FEASIBLE_SOLUTION_HEURISTIC_HPP_
#define GVRP_FEASIBLE_SOLUTION_HEURISTIC_HPP_

#include "models/gvrp_heuristic.hpp"
#include "models/gvrp_instance.hpp"
#include "models/gvrp_solution.hpp"

namespace models {
  class Gvrp_feasible_solution_heuristic : public Gvrp_heuristic {
    public:
      explicit Gvrp_feasible_solution_heuristic (Gvrp_instance& gvrp_instance);
      Gvrp_solution run();
  };
}

#endif
