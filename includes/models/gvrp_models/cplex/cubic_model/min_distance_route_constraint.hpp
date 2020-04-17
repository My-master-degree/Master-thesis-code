#ifndef MIN_DISTANCE_ROUTE_CONSTRAINT_CUBIC_MODEL_CPLEX_HPP_
#define MIN_DISTANCE_ROUTE_CONSTRAINT_CUBIC_MODEL_CPLEX_HPP_

#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp"
#include "models/gvrp_models/cplex/cubic_model/extra_constraint.hpp"

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace cubic_model {
        class Min_distance_route_constraint : public Extra_constraint{
          public:
            explicit Min_distance_route_constraint (Cubic_model& cubic_model);
            void add ();
        };
      }
    }
  }
}

#endif
