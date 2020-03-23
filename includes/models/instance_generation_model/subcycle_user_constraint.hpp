#ifndef SUBCYCLE_USER_CONSTRAINT_INSTANCE_GENERATION_HPP_
#define SUBCYCLE_USER_CONSTRAINT_INSTANCE_GENERATION_HPP_

#include "models/instance_generation_model/user_constraint.hpp"
#include "models/instance_generation_model/instance_generation_model.hpp"

using namespace models;

namespace models {
  namespace instance_generation_model {
    class Subcycle_user_constraint : public User_constraint {
      public:
        Subcycle_user_constraint (Instance_generation_model& instance_generation_model);
        [[nodiscard]] IloCplex::CallbackI* duplicateCallback() const override;
        void main() override;
    };
  }
}

#endif
