#ifndef ENERGY_LB_CONSTRAINT_EMH_MODEL_CPLEX_HPP_
#define ENERGY_LB_CONSTRAINT_EMH_MODEL_CPLEX_HPP_

#include "models/gvrp_models/cplex/emh_model/emh_model.hpp"
#include "models/gvrp_models/cplex/emh_model/extra_constraint.hpp"

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace emh_model {
        class Energy_lb_constraint : public Extra_constraint {
          public:
            explicit Energy_lb_constraint (EMH_model& emh_model);
            void add ();
        };
      }
    }
  }
}

#endif
