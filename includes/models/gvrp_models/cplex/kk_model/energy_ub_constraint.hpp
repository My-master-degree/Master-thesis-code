#ifndef ENERGY_UB_CONSTRAINT_KK_MODEL_CPLEX_HPP_
#define ENERGY_UB_CONSTRAINT_KK_MODEL_CPLEX_HPP_

#include "models/gvrp_models/cplex/kk_model/kk_model.hpp"
#include "models/gvrp_models/cplex/kk_model/extra_constraint.hpp"

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace kk_model {
        class Energy_ub_constraint : public Extra_constraint {
          public:
            explicit Energy_ub_constraint (KK_model& kk_model);
            void add ();
        };
      }
    }
  }
}

#endif
