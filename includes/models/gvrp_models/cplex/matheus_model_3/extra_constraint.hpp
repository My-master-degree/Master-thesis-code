#ifndef EXTRA_CONSTRAINT_MATHEUS_MODEL_3_CPLEX_HPP_
#define EXTRA_CONSTRAINT_MATHEUS_MODEL_3_CPLEX_HPP_

#include "models/gvrp_models/cplex/matheus_model_3/matheus_model_3.hpp"

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace matheus_model_3 {
        class Matheus_model_3;
        class Extra_constraint {
          protected:
            Matheus_model_3& matheus_model_3;
          public:
            explicit Extra_constraint (Matheus_model_3& matheus_model_3);
            ~Extra_constraint();
            virtual void add() = 0; 
        };
      }
    }
  }
}

#endif
