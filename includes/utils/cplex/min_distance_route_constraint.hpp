#ifndef MIN_DISTANCE_ROUTE_CONSTRAINT_HPP_
#define MIN_DISTANCE_ROUTE_CONSTRAINT_HPP_

#include "utils/cplex/compact_model.hpp"
#include "utils/cplex/extra_constraint_compact_model.hpp"

namespace utils {
  namespace cplex {
    class Min_distance_route_constraint : public Extra_constraint_compact_model{
      public:
        explicit Min_distance_route_constraint (Compact_model& compact_model, double lambda);
        void add ();
      private:
        double lambda;
    };
  }
}

#endif
