#ifndef USER_CONSTRAINT_INSTANCE_GENERATION_HPP_
#define USER_CONSTRAINT_INSTANCE_GENERATION_HPP_

#include "models/instance_generation_model/instance_generation_model.hpp" 

#include <ilcplex/ilocplex.h>

namespace models {
  namespace instance_generation_model {
    class Instance_generation_model;
    class User_constraint : public IloCplex::UserCutCallbackI {
      protected:
        Instance_generation_model& instance_generation_model;
        const double EPS;
      public:
         explicit User_constraint (Instance_generation_model& instance_generation_model);
    };    
  }
} 

#endif
