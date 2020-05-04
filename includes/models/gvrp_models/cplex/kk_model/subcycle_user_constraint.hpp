#ifndef SUBCYCLE_USER_CONSTRAINT_KK_MODEL_CPLEX_HPP_
#define SUBCYCLE_USER_CONSTRAINT_KK_MODEL_CPLEX_HPP_

#include "models/gvrp_models/cplex/kk_model/user_constraint.hpp" 
#include "models/gvrp_models/cplex/kk_model/kk_model.hpp" 

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models;

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace kk_model {
        class Subcycle_user_constraint : public User_constraint {
          public:
            Subcycle_user_constraint (KK_model& kk_model);
            [[nodiscard]] IloCplex::CallbackI* duplicateCallback() const override;
            void main() override;
        };
      } 
    } 
  } 
}
#endif
