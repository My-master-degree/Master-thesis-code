#ifndef USER_CONSTRAINT_LH_MODEL_CPLEX_HPP_
#define USER_CONSTRAINT_LH_MODEL_CPLEX_HPP_

#include "models/gvrp_models/cplex/lh_model/lh_model.hpp" 

#include <ilcplex/ilocplex.h>

namespace models{
  namespace gvrp_models {
    namespace cplex {
      namespace lh_model {
        class LH_model;
        class User_constraint : public IloCplex::UserCutCallbackI {
          protected:
            LH_model& lh_model;
            const double EPS;
          public:
            explicit User_constraint (LH_model& lh_model);
        };    
      }
    }
  }
} 
#endif 
