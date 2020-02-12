#include "utils/cplex/compact_model.hpp"
#include "utils/cplex/extra_constraint_compact_model.hpp"
#include "utils/cplex/invalid_paths_constraint.hpp"

#include <ilcplex/ilocplex.h>

using namespace utils::cplex;

Invalid_paths_constraint::Invalid_paths_constraint (Compact_model& compact_model) : Extra_constraint_compact_model (compact_model) {}

void Invalid_paths_constraint::add (){
  IloExpr lhs (compact_model.env),
          rhs (compact_model.env);
  IloConstraint c;
}
