#ifndef EXTRA_CONSTRAINT_MATHEUS_MODEL_CPLEX_HPP_
#define EXTRA_CONSTRAINT_MATHEUS_MODEL_CPLEX_HPP_

#include "models/gvrp_models/cplex/matheus_model/matheus_model.hpp"

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace matheus_model {
        class Matheus_model;
        class Extra_constraint {
          protected:
            Matheus_model& matheus_model;
          public:
            explicit Extra_constraint (Matheus_model& matheus_model);
            virtual void add() = 0; 
        };
      }
    }
  }
}

#endif
