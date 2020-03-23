#include "models/instance_generation_model/instance_generation_model.hpp"
#include "models/instance_generation_model/user_constraint.hpp"

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models::instance_generation_model;

User_constraint::User_constraint (Instance_generation_model& instance_generation_model_): UserCutCallbackI (instance_generation_model_.env), instance_generation_model(instance_generation_model_), EPS(1e-3) {}
