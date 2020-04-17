#include "models/gvrp_models/cplex/emh_model/emh_model.hpp"
#include "models/gvrp_models/cplex/emh_model/user_constraint.hpp"

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models::gvrp_models::cplex::emh_model;

User_constraint::User_constraint (EMH_model& emh_model_): UserCutCallbackI (emh_model_.env), emh_model(emh_model_), EPS(1e-3) { }
