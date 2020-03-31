#ifndef USER_CONSTRAINT_EMH_MODEL_HPP_
#define USER_CONSTRAINT_EMH_MODEL_HPP_

#include "models/emh_model/emh_model.hpp" 

#include <ilcplex/ilocplex.h>

namespace models{
  namespace emh_model {
    class EMH_model;
    class User_constraint_emh_model : public IloCplex::UserCutCallbackI {
      protected:
        EMH_model& emh_model;
        const double EPS;
      public:
         explicit User_constraint_emh_model (EMH_model& emh_model);
    };    
  }
} 
#endif 
