#include "models/gvrp_models/cplex/matheus_model/matheus_model.hpp"
#include "models/gvrp_models/cplex/matheus_model/user_constraint.hpp"

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models::gvrp_models::cplex::matheus_model;

User_constraint::User_constraint (Matheus_model& matheus_model_): UserCutCallbackI (matheus_model_.env), matheus_model(matheus_model_), EPS(1e-3) { }
