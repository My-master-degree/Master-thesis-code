#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp"
#include "models/gvrp_models/cplex/cubic_model/extra_constraint.hpp"
#include "models/gvrp_models/cplex/cubic_model/invalid_paths_constraint.hpp"

#include <ilcplex/ilocplex.h>

using namespace models::gvrp_models::cplex::cubic_model;

Invalid_paths_constraint::Invalid_paths_constraint (Cubic_model& cubic_model) : Extra_constraint (cubic_model) {}

void Invalid_paths_constraint::add (){
  IloExpr lhs (cubic_model.env),
          rhs (cubic_model.env);
  IloConstraint c;
}
