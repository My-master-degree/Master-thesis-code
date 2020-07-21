#include "models/gvrp_models/cplex/matheus_model_3/matheus_model_3.hpp"
#include "models/gvrp_models/cplex/matheus_model_3/lazy_constraint.hpp"

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models::gvrp_models::cplex::matheus_model_3;

Lazy_constraint::Lazy_constraint (Matheus_model_3& matheus_model_3_): LazyConstraintCallbackI (matheus_model_3_.env), matheus_model_3(matheus_model_3_) { }
