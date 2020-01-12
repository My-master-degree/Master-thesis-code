#ifndef USER_CONSTRAINT_COMPACT_MODEL_HPP_
#define USER_CONSTRAINT_COMPACT_MODEL_HPP_

#include "utils/cplex/compact_model.hpp" 

#include <ilcplex/ilocplex.h>

namespace utils{
  namespace cplex {
    class Compact_model;
    class User_constraint_compact_model : public IloCplex::UserCutCallbackI {
      protected:
        Compact_model& compact_model;
        const double EPS;
      public:
         explicit User_constraint_compact_model (Compact_model& compact_model);
    };    
  }
} 
#endif 
