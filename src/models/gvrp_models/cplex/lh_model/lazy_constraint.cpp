#include "models/gvrp_models/cplex/lh_model/lh_model.hpp"
#include "models/gvrp_models/cplex/lh_model/lazy_constraint.hpp"

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models::gvrp_models::cplex::lh_model;

Lazy_constraint::Lazy_constraint (LH_model& lh_model_): LazyConstraintCallbackI (lh_model_.env), lh_model(lh_model_) { }
