#ifndef GVRP_MODEL_HPP_
#define GVRP_MODEL_HPP_

#include "models/gvrp_instance.hpp"
#include "models/gvrp_solution.hpp"
#include "models/mip_solution_info.hpp"

#include <ilcplex/ilocplex.h>

ILOSTLBEGIN

namespace models {
  class Gvrp_model {
    public:
      explicit Gvrp_model(Gvrp_instance& gvrp_instance, unsigned int time_limit); 
      Gvrp_instance gvrp_instance;
      unsigned int time_limit;//seconds
      unsigned int max_num_feasible_integer_sol;//0 to 2100000000
      bool VERBOSE;
      Gvrp_solution* gvrp_solution;
      IloEnv env;
      IloModel model;
      IloCplex cplex;
      virtual pair<Gvrp_solution, Mip_solution_info> run() = 0;
  };
}
#endif
