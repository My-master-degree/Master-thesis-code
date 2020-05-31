#include "models/gvrp_models/cplex/matheus_model/matheus_model.hpp"
#include "models/gvrp_models/cplex/matheus_model/lazy_constraint.hpp"

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models::gvrp_models::cplex::matheus_model;

Lazy_constraint::Lazy_constraint (Matheus_model& matheus_model_): LazyConstraintCallbackI (matheus_model_.env), matheus_model(matheus_model_) { }
