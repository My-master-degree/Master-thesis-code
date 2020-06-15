#ifndef ENERGY_UB_CONSTRAINT_MATHEUS_MODEL_2_CPLEX_HPP_
#define ENERGY_UB_CONSTRAINT_MATHEUS_MODEL_2_CPLEX_HPP_

#include "models/gvrp_models/cplex/matheus_model_2/matheus_model_2.hpp"
#include "models/gvrp_models/cplex/matheus_model_2/extra_constraint.hpp"

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace matheus_model_2 {
        class Energy_ub_constraint : public Extra_constraint {
          public:
            explicit Energy_ub_constraint (Matheus_model_2& matheus_model_2);
            void add ();
        };
      }
    }
  }
}

#endif
