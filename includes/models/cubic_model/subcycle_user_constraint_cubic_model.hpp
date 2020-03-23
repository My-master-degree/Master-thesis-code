#ifndef SUBCYCLE_USER_CONSTRAINT_CUBIC_MODEL_HPP_
#define SUBCYCLE_USER_CONSTRAINT_CUBIC_MODEL_HPP_

#include "models/cubic_model/user_constraint_cubic_model.hpp" 
#include "models/cubic_model/cubic_model.hpp" 

#include <ilcplex/ilocplex.h>
//ILOSTLBEGIN

using namespace std;
using namespace models;

namespace models {
  namespace cubic_model {
    class Subcycle_user_constraint_cubic_model : public User_constraint_cubic_model {
      public:
         Subcycle_user_constraint_cubic_model (Cubic_model& cubic_model);
         [[nodiscard]] IloCplex::CallbackI* duplicateCallback() const override;
         void main() override;
    };
  } 
}
#endif
