#include "models/cubic_model/cubic_model.hpp"
#include "models/cubic_model/lazy_constraint_cubic_model.hpp"

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models::cubic_model;

Lazy_constraint_cubic_model::Lazy_constraint_cubic_model (Cubic_model& cubic_model_): LazyConstraintCallbackI (cubic_model_.env), cubic_model(cubic_model_) { }
