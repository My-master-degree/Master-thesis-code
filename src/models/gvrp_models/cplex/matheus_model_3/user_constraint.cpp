#include "models/gvrp_models/cplex/matheus_model_3/matheus_model_3.hpp"
#include "models/gvrp_models/cplex/matheus_model_3/user_constraint.hpp"

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models::gvrp_models::cplex::matheus_model_3;

User_constraint::User_constraint (Matheus_model_3& matheus_model_3_): UserCutCallbackI (matheus_model_3_.env), matheus_model_3(matheus_model_3_), EPS(1e-3) { }
