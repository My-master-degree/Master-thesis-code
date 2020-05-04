#ifndef EXTRA_CONSTRAINT_KK_MODEL_CPLEX_HPP_
#define EXTRA_CONSTRAINT_KK_MODEL_CPLEX_HPP_

#include "models/gvrp_models/cplex/kk_model/kk_model.hpp"

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace kk_model {
        class KK_model;
        class Extra_constraint {
          protected:
            KK_model& kk_model;
          public:
            explicit Extra_constraint (KK_model& kk_model);
            virtual void add() = 0; 
        };
      }
    }
  }
}

#endif
