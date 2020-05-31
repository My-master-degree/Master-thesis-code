#ifndef ENERGY_LB_CONSTRAINT_MATHEUS_MODEL_CPLEX_HPP_
#define ENERGY_LB_CONSTRAINT_MATHEUS_MODEL_CPLEX_HPP_

#include "models/gvrp_models/cplex/matheus_model/matheus_model.hpp"
#include "models/gvrp_models/cplex/matheus_model/extra_constraint.hpp"

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace matheus_model {
        class Energy_lb_constraint : public Extra_constraint {
          public:
            explicit Energy_lb_constraint (Matheus_model& matheus_models);
            void add ();
        };
      }
    }
  }
}

#endif
