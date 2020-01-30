#include "utils/cplex/compact_model.hpp"
#include "utils/cplex/extra_constraint_compact_model.hpp"
#include "utils/cplex/custom_test_constraint.hpp"

#include <ilcplex/ilocplex.h>

using namespace utils::cplex;

Custom_test_constraint::Custom_test_constraint (Compact_model& compact_model, Gvrp_solution& gvrp_solution_) : Extra_constraint_compact_model (compact_model), gvrp_solution (gvrp_solution_) {}

void Custom_test_constraint::add () {

}
