#include "models/gvrp_models/cplex/emh_model/emh_model.hpp"
#include "models/gvrp_models/cplex/emh_model/lazy_constraint.hpp"

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models::gvrp_models::cplex::emh_model;

Lazy_constraint::Lazy_constraint (EMH_model& emh_model_): LazyConstraintCallbackI (emh_model_.env), emh_model(emh_model_) { }
