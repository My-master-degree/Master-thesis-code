#include "models/gvrp_models/cplex/lh_model/lh_model.hpp"
#include "models/gvrp_models/cplex/lh_model/user_constraint.hpp"

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models::gvrp_models::cplex::lh_model;

User_constraint::User_constraint (LH_model& lh_model_): UserCutCallbackI (lh_model_.env), lh_model(lh_model_), EPS(1e-3) { }
