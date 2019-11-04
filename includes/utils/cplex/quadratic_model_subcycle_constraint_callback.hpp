#ifndef QUADRATIC_MODEL_SUBCYCLE_CONSTRAINT_CALLBACK_HPP_
#define QUADRATIC_MODEL_SUBCYCLE_CONSTRAINT_CALLBACK_HPP_

#include <ilcplex/ilocplex.h>

using namespace std;

namespace quadratic_model_subcycle_constraint_callback {
  class Quadratic_model_subcycle_constraint_callback : public IloCplex::LazyConstraintCallbackI {
    public:
      explicit Quadratic_model_subcycle_constraint_callback(IloEnv env, IloExprArray xx1, IloNumArray xx2, IloNum xx3); 
      IloCplex::CallbackI* duplicateCallback() const;
      void main(void);
      IloExprArray lhs;
      IloNumArray rhs;
      IloNum eps;
 };

} 
#endif
