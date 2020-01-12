#include "utils/cplex/compact_model.hpp"
#include "utils/cplex/lazy_constraint_compact_model.hpp"

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace utils::cplex;

Lazy_constraint_compact_model::Lazy_constraint_compact_model (Compact_model& compact_model_): LazyConstraintCallbackI (compact_model_.env), compact_model(compact_model_) { }
