#ifndef SUBCYCLE_USER_CONSTRAINT_LH_MODEL_CPLEX_HPP_
#define SUBCYCLE_USER_CONSTRAINT_LH_MODEL_CPLEX_HPP_

#include "models/gvrp_models/cplex/lh_model/user_constraint.hpp" 
#include "models/gvrp_models/cplex/lh_model/lh_model.hpp" 

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models;

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace lh_model {
        class Subcycle_user_constraint : public User_constraint {
          public:
            Subcycle_user_constraint (LH_model& lh_model);
            [[nodiscard]] IloCplex::CallbackI* duplicateCallback() const override;
            void main() override;
        };
      } 
    } 
  } 
}
#endif
