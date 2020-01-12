#ifndef LAZY_CONSTRAINT_COMPACT_MODEL_HPP_
#define LAZY_CONSTRAINT_COMPACT_MODEL_HPP_

#include "utils/cplex/compact_model.hpp" 

#include <ilcplex/ilocplex.h>

namespace utils{
  namespace cplex {
    class Compact_model;
    class Lazy_constraint_compact_model : public IloCplex::LazyConstraintCallbackI {
      protected:
        Compact_model& compact_model;
      public:
         explicit Lazy_constraint_compact_model (Compact_model& compact_model);
    };    
  }
} 
#endif 
