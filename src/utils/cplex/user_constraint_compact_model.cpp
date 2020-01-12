#include "utils/cplex/compact_model.hpp"
#include "utils/cplex/user_constraint_compact_model.hpp"

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace utils::cplex;

User_constraint_compact_model::User_constraint_compact_model (Compact_model& compact_model_): UserCutCallbackI (compact_model_.env), compact_model(compact_model_), EPS(1e-3) { }
