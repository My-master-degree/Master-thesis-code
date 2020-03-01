#include "models/cubic_model/cubic_model.hpp"
#include "models/cubic_model/user_constraint_cubic_model.hpp"

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models::cubic_model;

User_constraint_cubic_model::User_constraint_cubic_model (Cubic_model& cubic_model_): UserCutCallbackI (cubic_model_.env), cubic_model(cubic_model_), EPS(1e-3) { }
