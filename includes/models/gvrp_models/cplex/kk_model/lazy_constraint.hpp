#ifndef LAZY_CONSTRAINT_KK_MODEL_CPLEX_HPP_
#define LAZY_CONSTRAINT_KK_MODEL_CPLEX_HPP_

#include "models/gvrp_models/cplex/kk_model/kk_model.hpp" 

#include <ilcplex/ilocplex.h>

namespace models{
  namespace gvrp_models {
    namespace cplex {
      namespace kk_model {
        class KK_model;
        class Lazy_constraint : public IloCplex::LazyConstraintCallbackI {
          protected:
            KK_model& kk_model;
          public:
            explicit Lazy_constraint (KK_model& kk_model);
        };    
      }
    }
  }
} 
#endif 
