#ifndef LAZY_CONSTRAINT_CUBIC_MODEL_HPP_
#define LAZY_CONSTRAINT_CUBIC_MODEL_HPP_

#include "models/cubic_model/cubic_model.hpp" 

#include <ilcplex/ilocplex.h>

namespace models{
  namespace cubic_model {
    class Cubic_model;
    class Lazy_constraint_cubic_model : public IloCplex::LazyConstraintCallbackI {
      protected:
        Cubic_model& cubic_model;
      public:
         explicit Lazy_constraint_cubic_model (Cubic_model& cubic_model);
    };    
  }
} 
#endif 
