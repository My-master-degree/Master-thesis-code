#include "models/instance_generation_model/instance_generation_model.hpp"
#include "models/instance_generation_model/lazy_constraint.hpp"

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models::instance_generation_model;

Lazy_constraint::Lazy_constraint (Instance_generation_model& instance_generation_model_): LazyConstraintCallbackI (instance_generation_model_.env), instance_generation_model(instance_generation_model_) { }
