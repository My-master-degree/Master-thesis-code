#ifndef LAZY_CONSTRAINT_MATHEUS_MODEL_3_CPLEX_HPP_
#define LAZY_CONSTRAINT_MATHEUS_MODEL_3_CPLEX_HPP_

#include "models/gvrp_models/cplex/matheus_model_3/matheus_model_3.hpp" 

#include <ilcplex/ilocplex.h>

namespace models{
  namespace gvrp_models {
    namespace cplex {
      namespace matheus_model_3 {
        class Matheus_model_3;
        class Lazy_constraint : public IloCplex::LazyConstraintCallbackI {
          protected:
            Matheus_model_3& matheus_model_3;
          public:
            explicit Lazy_constraint (Matheus_model_3& matheus_model_3);
        };    
      }
    }
  }
} 
#endif 
