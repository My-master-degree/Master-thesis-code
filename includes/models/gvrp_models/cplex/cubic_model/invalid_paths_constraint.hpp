#ifndef INVALID_PATHS_CONSTRAINT_HPP_
#define INVALID_PATHS_CONSTRAINT_HPP_ 

#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp"
#include "models/gvrp_models/cplex/cubic_model/extra_constraint.hpp"

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace cubic_model {
        class Invalid_paths_constraint : public Extra_constraint {
          public:
            explicit Invalid_paths_constraint (Cubic_model& cubic_model);
            void add ();
        };
      }
    }
  }
}

#endif
