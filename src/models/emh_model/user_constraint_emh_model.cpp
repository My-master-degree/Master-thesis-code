#include "models/emh_model/emh_model.hpp"
#include "models/emh_model/user_constraint_emh_model.hpp"

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models::emh_model;

User_constraint_emh_model::User_constraint_emh_model (EMH_model& emh_model_): UserCutCallbackI (emh_model_.env), emh_model(emh_model_), EPS(1e-3) { }
