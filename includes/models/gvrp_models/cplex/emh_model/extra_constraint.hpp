#ifndef EXTRA_CONSTRAINT_EMH_MODEL_CPLEX_HPP_
#define EXTRA_CONSTRAINT_EMH_MODEL_CPLEX_HPP_

#include "models/gvrp_models/cplex/emh_model/emh_model.hpp"

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace emh_model {
        class EMH_model;
        class Extra_constraint {
          protected:
            EMH_model& emh_model;
          public:
            explicit Extra_constraint (EMH_model& emh_model);
            ~Extra_constraint();
            virtual void add() = 0; 
        };
      }
    }
  }
}

#endif
