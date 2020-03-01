#ifndef USER_CONSTRAINT_CUBIC_MODEL_HPP_
#define USER_CONSTRAINT_CUBIC_MODEL_HPP_

#include "models/cubic_model/cubic_model.hpp" 

#include <ilcplex/ilocplex.h>

namespace models{
  namespace cubic_model {
    class Cubic_model;
    class User_constraint_cubic_model : public IloCplex::UserCutCallbackI {
      protected:
        Cubic_model& cubic_model;
        const double EPS;
      public:
         explicit User_constraint_cubic_model (Cubic_model& cubic_model);
    };    
  }
} 
#endif 
