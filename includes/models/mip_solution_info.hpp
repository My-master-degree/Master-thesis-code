#ifndef MIP_SOLUTION_INFO_HPP_
#define MIP_SOLUTION_INFO_HPP_

namespace models {
  class Mip_solution_info{
    double gap;
    public:
      explicit Mip_solution_info (double gap_);
  };
}
#endif
