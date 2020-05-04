#include "models/gvrp_models/cplex/kk_model/kk_model.hpp"
#include "models/gvrp_models/cplex/kk_model/user_constraint.hpp"

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models::gvrp_models::cplex::kk_model;

User_constraint::User_constraint (KK_model& kk_model_): UserCutCallbackI (kk_model_.env), kk_model(kk_model_), EPS(1e-3) { }
