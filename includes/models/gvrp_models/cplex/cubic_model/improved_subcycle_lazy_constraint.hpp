#ifndef IMPROVED_SUBCYCLE_LAZY_CONSTRAINT_CUBIC_MODEL_CPLEX_HPP_
#define IMPROVED_SUBCYCLE_LAZY_CONSTRAINT_CUBIC_MODEL_CPLEX_HPP_

#include "models/gvrp_models/cplex/cubic_model/lazy_constraint.hpp" 
#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp" 

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models;

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace cubic_model {
        class Improved_subcycle_lazy_constraint : public Lazy_constraint {
          public:
            Improved_subcycle_lazy_constraint (Cubic_model& cubic_model);
            [[nodiscard]] IloCplex::CallbackI* duplicateCallback() const override;
            void main() override;
        };
      } 
    } 
  } 
}
#endif
