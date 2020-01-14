#ifndef ROUTES_LB_CONSTRAINT_HPP_
#define ROUTES_LB_CONSTRAINT_HPP_

#include "utils/cplex/compact_model.hpp"
#include "utils/cplex/extra_constraint_compact_model.hpp"

namespace utils {
  namespace cplex {
    class Routes_lb_constraint : public Extra_constraint_compact_model {
      public:
        explicit Routes_lb_constraint (Compact_model& compact_model);
        void add ();
    };
  }
}

#endif
