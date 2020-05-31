#ifndef LAZY_CONSTRAINT_MATHEUS_MODEL_CPLEX_HPP_
#define LAZY_CONSTRAINT_MATHEUS_MODEL_CPLEX_HPP_

#include "models/gvrp_models/cplex/matheus_model/matheus_model.hpp" 

#include <ilcplex/ilocplex.h>

namespace models{
  namespace gvrp_models {
    namespace cplex {
      namespace matheus_model {
        class Matheus_model;
        class Lazy_constraint : public IloCplex::LazyConstraintCallbackI {
          protected:
            Matheus_model& matheus_model;
          public:
            explicit Lazy_constraint (Matheus_model& matheus_model);
        };    
      }
    }
  }
} 
#endif 
