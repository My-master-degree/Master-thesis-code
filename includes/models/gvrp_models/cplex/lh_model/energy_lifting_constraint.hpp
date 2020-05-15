#ifndef ENERGY_LIFTING_CONSTRAINT_LH_MODEL_CPLEX_HPP_
#define ENERGY_LIFTING_CONSTRAINT_LH_MODEL_CPLEX_HPP_

#include "models/gvrp_models/cplex/lh_model/lh_model.hpp"
#include "models/gvrp_models/cplex/lh_model/extra_constraint.hpp"

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace lh_model {
        class Energy_lifting_constraint : public Extra_constraint {
          public:
            explicit Energy_lifting_constraint (LH_model& lh_model);
            void add ();
        };
      }
    }
  }
}

#endif
