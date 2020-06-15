#include "models/gvrp_models/cplex/matheus_model_2/matheus_model_2.hpp"
#include "models/gvrp_models/cplex/matheus_model_2/lazy_constraint.hpp"

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models::gvrp_models::cplex::matheus_model_2;

Lazy_constraint::Lazy_constraint (Matheus_model_2& matheus_model_2_): LazyConstraintCallbackI (matheus_model_2_.env), matheus_model_2(matheus_model_2_) { }
