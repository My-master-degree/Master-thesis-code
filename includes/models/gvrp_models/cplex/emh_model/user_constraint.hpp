#ifndef USER_CONSTRAINT_EMH_MODEL_CPLEX_HPP_
#define USER_CONSTRAINT_EMH_MODEL_CPLEX_HPP_

#include "models/gvrp_models/cplex/emh_model/emh_model.hpp" 

#include <ilcplex/ilocplex.h>

namespace models{
  namespace gvrp_models {
    namespace cplex {
      namespace emh_model {
        class EMH_model;
        class User_constraint : public IloCplex::UserCutCallbackI {
          protected:
            EMH_model& emh_model;
            const double EPS;
          public:
            explicit User_constraint (EMH_model& emh_model);
        };    
      }
    }
  }
} 
#endif 
