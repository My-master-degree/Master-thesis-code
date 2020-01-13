#ifndef ENERGY_LB_CONSTRAINT_HPP_
#define ENERGY_LB_CONSTRAINT_HPP_

#include "utils/cplex/compact_model.hpp"
#include "utils/cplex/extra_constraint_compact_model.hpp"

namespace utils {
  namespace cplex {
    class Energy_lb_constraint : public Extra_constraint_compact_model {
      public:
        explicit Energy_lb_constraint (Compact_model& compact_model);
        void add ();
    };
  }
}

#endif
