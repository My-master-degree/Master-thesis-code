#include "models/gvrp_models/cplex/kk_model/kk_model.hpp"
#include "models/gvrp_models/cplex/kk_model/lazy_constraint.hpp"

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models::gvrp_models::cplex::kk_model;

Lazy_constraint::Lazy_constraint (KK_model& kk_model_): LazyConstraintCallbackI (kk_model_.env), kk_model(kk_model_) { }
