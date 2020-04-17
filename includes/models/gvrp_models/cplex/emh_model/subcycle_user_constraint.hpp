#ifndef SUBCYCLE_USER_CONSTRAINT_EMH_MODEL_CPLEX_HPP_
#define SUBCYCLE_USER_CONSTRAINT_EMH_MODEL_CPLEX_HPP_

#include "models/gvrp_models/cplex/emh_model/user_constraint.hpp" 
#include "models/gvrp_models/cplex/emh_model/emh_model.hpp" 

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models;

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace emh_model {
        class Subcycle_user_constraint : public User_constraint {
          public:
            Subcycle_user_constraint (EMH_model& emh_model);
            [[nodiscard]] IloCplex::CallbackI* duplicateCallback() const override;
            void main() override;
        };
      } 
    } 
  } 
}
#endif
