#ifndef SUBCYCLE_LAZY_CONSTRAINT_INSTANCE_GENERATION_HPP_
#define SUBCYCLE_LAZY_CONSTRAINT_INSTANCE_GENERATION_HPP_

#include "models/instance_generation_model/lazy_constraint.hpp" 

#include <ilcplex/ilocplex.h>

namespace models {
  namespace instance_generation_model {
    class Subcycle_lazy_constraint : public Lazy_constraint {
      public:
        explicit Subcycle_lazy_constraint(Instance_generation_model& instance_generation_model);
        [[nodiscard]] IloCplex::CallbackI* duplicateCallback() const override;
        void main() override;
    };
  }
}

#endif
