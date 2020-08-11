#ifndef SUBCYCLE_USER_CONSTRAINT_MATHEUS_MODEL_5_CPLEX_HPP_
#define SUBCYCLE_USER_CONSTRAINT_MATHEUS_MODEL_5_CPLEX_HPP_

#include "models/gvrp_models/cplex/matheus_model_5/user_constraint.hpp" 
#include "models/gvrp_models/cplex/matheus_model_5/matheus_model_5.hpp" 

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models;

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace matheus_model_5 {
        class Subcycle_user_constraint : public User_constraint {
          public:
            Subcycle_user_constraint (Matheus_model_5& matheus_model_5);
            [[nodiscard]] IloCplex::CallbackI* duplicateCallback() const override;
            void main() override;
        };
      } 
    } 
  } 
}
#endif
