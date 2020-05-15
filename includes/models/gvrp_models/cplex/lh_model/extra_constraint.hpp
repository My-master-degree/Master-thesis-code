#ifndef EXTRA_CONSTRAINT_LH_MODEL_CPLEX_HPP_
#define EXTRA_CONSTRAINT_LH_MODEL_CPLEX_HPP_

#include "models/gvrp_models/cplex/lh_model/lh_model.hpp"

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace lh_model {
        class LH_model;
        class Extra_constraint {
          protected:
            LH_model& lh_model;
          public:
            explicit Extra_constraint (LH_model& lh_model);
            virtual void add() = 0; 
        };
      }
    }
  }
}

#endif
