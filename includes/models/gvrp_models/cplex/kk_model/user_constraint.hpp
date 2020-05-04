#ifndef USER_CONSTRAINT_KK_MODEL_CPLEX_HPP_
#define USER_CONSTRAINT_KK_MODEL_CPLEX_HPP_

#include "models/gvrp_models/cplex/kk_model/kk_model.hpp" 

#include <ilcplex/ilocplex.h>

namespace models{
  namespace gvrp_models {
    namespace cplex {
      namespace kk_model {
        class KK_model;
        class User_constraint : public IloCplex::UserCutCallbackI {
          protected:
            KK_model& kk_model;
            const double EPS;
          public:
            explicit User_constraint (KK_model& kk_model);
        };    
      }
    }
  }
} 
#endif 
