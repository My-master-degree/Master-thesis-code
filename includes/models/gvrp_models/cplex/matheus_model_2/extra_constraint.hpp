#ifndef EXTRA_CONSTRAINT_MATHEUS_MODEL_2_CPLEX_HPP_
#define EXTRA_CONSTRAINT_MATHEUS_MODEL_2_CPLEX_HPP_

#include "models/gvrp_models/cplex/matheus_model_2/matheus_model_2.hpp"

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace matheus_model_2 {
        class Matheus_model_2;
        class Extra_constraint {
          protected:
            Matheus_model_2& matheus_model_2;
          public:
            explicit Extra_constraint (Matheus_model_2& matheus_model_2);
            virtual void add() = 0; 
        };
      }
    }
  }
}

#endif
