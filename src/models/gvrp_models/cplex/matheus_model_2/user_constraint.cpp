#include "models/gvrp_models/cplex/matheus_model_2/matheus_model_2.hpp"
#include "models/gvrp_models/cplex/matheus_model_2/user_constraint.hpp"

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models::gvrp_models::cplex::matheus_model_2;

User_constraint::User_constraint (Matheus_model_2& matheus_model_2_): UserCutCallbackI (matheus_model_2_.env), matheus_model_2(matheus_model_2_), EPS(1e-3) { }
