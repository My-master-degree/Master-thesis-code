#ifndef IMPROVED_SUBCYCLE_LAZY_CONSTRAINT_CUBIC_MODEL_HPP_
#define IMPROVED_SUBCYCLE_LAZY_CONSTRAINT_CUBIC_MODEL_HPP_

#include "models/cubic_model/lazy_constraint_cubic_model.hpp" 
#include "models/cubic_model/cubic_model.hpp" 

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models;

namespace models {
  namespace cubic_model {
    class Improved_subcycle_lazy_constraint_cubic_model : public Lazy_constraint_cubic_model {
      public:
         Improved_subcycle_lazy_constraint_cubic_model (Cubic_model& cubic_model);
         [[nodiscard]] IloCplex::CallbackI* duplicateCallback() const override;
         void main() override;
    };
  } 
}
#endif
