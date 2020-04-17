#ifndef LAZY_CONSTRAINT_EMH_MODEL_CPLEX_HPP_
#define LAZY_CONSTRAINT_EMH_MODEL_CPLEX_HPP_

#include "models/gvrp_models/cplex/emh_model/emh_model.hpp" 

#include <ilcplex/ilocplex.h>

namespace models{
  namespace gvrp_models {
    namespace cplex {
      namespace emh_model {
        class EMH_model;
        class Lazy_constraint : public IloCplex::LazyConstraintCallbackI {
          protected:
            EMH_model& emh_model;
          public:
            explicit Lazy_constraint (EMH_model& emh_model);
        };    
      }
    }
  }
} 
#endif 
