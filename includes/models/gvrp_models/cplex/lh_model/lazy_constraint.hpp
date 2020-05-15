#ifndef LAZY_CONSTRAINT_LH_MODEL_CPLEX_HPP_
#define LAZY_CONSTRAINT_LH_MODEL_CPLEX_HPP_

#include "models/gvrp_models/cplex/lh_model/lh_model.hpp" 

#include <ilcplex/ilocplex.h>

namespace models{
  namespace gvrp_models {
    namespace cplex {
      namespace lh_model {
        class LH_model;
        class Lazy_constraint : public IloCplex::LazyConstraintCallbackI {
          protected:
            LH_model& lh_model;
          public:
            explicit Lazy_constraint (LH_model& lh_model);
        };    
      }
    }
  }
} 
#endif 
