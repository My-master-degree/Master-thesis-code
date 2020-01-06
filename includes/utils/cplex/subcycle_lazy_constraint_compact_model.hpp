#ifndef SUBCYCLE_LAZY_CONSTRAINT_COMPACT_MODEL_HPP_
#define SUBCYCLE_LAZY_CONSTRAINT_COMPACT_MODEL_HPP_

#include "utils/cplex/compact_model.hpp" 

#include <ilcplex/ilocplex.h>
//ILOSTLBEGIN

using namespace std;
using namespace models;
//using namespace utils;

namespace utils {
  namespace cplex {
    class Compact_model;
    class Subcycle_lazy_constraint_compact_model : public IloCplex::LazyConstraintCallbackI {
      private:
        Compact_model& compact_model;
      public:
         Subcycle_lazy_constraint_compact_model (Compact_model& compact_model);
         [[nodiscard]] IloCplex::CallbackI* duplicateCallback() const override;
         void main() override;
    };
  } 
}
#endif
