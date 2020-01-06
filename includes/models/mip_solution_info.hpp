#ifndef MIP_SOLUTION_INFO_HPP_
#define MIP_SOLUTION_INFO_HPP_

#include <ilcplex/ilocplex.h>

namespace models {
  class Mip_solution_info{
    double gap;
    IloAlgorithm::Status status;
    public:
      explicit Mip_solution_info (double gap_, IloAlgorithm::Status status_);
  };
}
#endif
