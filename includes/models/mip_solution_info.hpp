#ifndef MIP_SOLUTION_INFO_HPP_
#define MIP_SOLUTION_INFO_HPP_

#include <ilcplex/ilocplex.h>

namespace models {
  class Mip_solution_info{
    public:
      double gap;
      IloAlgorithm::Status status;
      double elapsed_time;
      double cost;
      explicit Mip_solution_info ();
      explicit Mip_solution_info (double gap_, IloAlgorithm::Status status_, double elapsed_time, double cost);
  };
}
#endif
