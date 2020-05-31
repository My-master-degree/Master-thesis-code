#ifndef USER_CONSTRAINT_MATHEUS_MODEL_CPLEX_HPP_
#define USER_CONSTRAINT_MATHEUS_MODEL_CPLEX_HPP_

#include "models/gvrp_models/cplex/matheus_model/matheus_model.hpp" 

#include <ilcplex/ilocplex.h>

namespace models{
  namespace gvrp_models {
    namespace cplex {
      namespace matheus_model {
        class Matheus_model;
        class User_constraint : public IloCplex::UserCutCallbackI {
          protected:
            Matheus_model& matheus_model;
            const double EPS;
          public:
            explicit User_constraint (Matheus_model& matheus_model);
        };    
      }
    }
  }
} 
#endif 
