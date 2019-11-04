#include "utils/cplex/quadratic_model_subcycle_constraint_callback.hpp"

#include <ilcplex/ilocplex.h>

ILOSTLBEGIN
using namespace std;

using quadratic_model_subcycle_constraint_callback::Quadratic_model_subcycle_constraint_callback;

Quadratic_model_subcycle_constraint_callback::Quadratic_model_subcycle_constraint_callback(IloEnv env, IloExprArray xx1, IloNumArray xx2, IloNum xx3) : 
  IloCplex::LazyConstraintCallbackI(env), lhs(xx1), rhs(xx2), eps(xx3) {
}

IloCplex::CallbackI* Quadratic_model_subcycle_constraint_callback::duplicateCallback() const{
  return (new(getEnv()) Quadratic_model_subcycle_constraint_callback(*this));
} 
void Quadratic_model_subcycle_constraint_callback::main(void){

}
