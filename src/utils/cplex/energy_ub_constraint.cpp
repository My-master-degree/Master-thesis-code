#include "utils/cplex/compact_model.hpp"
#include "utils/cplex/extra_constraint_compact_model.hpp"
#include "utils/cplex/energy_ub_constraint.hpp"
#include "utils/util.hpp"

#include <float.h>
#include <map>
#include <ilcplex/ilocplex.h>

using namespace std;
using namespace utils;
using namespace utils::cplex;

Energy_ub_constraint::Energy_ub_constraint (Compact_model& compact_model) : Extra_constraint_compact_model (compact_model){
}

void Energy_ub_constraint::add () {
  map<int, double> customersEnergyUBs = calculateCustomersEnergyUB(compact_model);
  IloConstraint c;
  for (Vertex customer : compact_model.gvrp_instance.customers){
    int i = customer.id;
    //e_i \leqslant minEdge
    c = IloConstraint (compact_model.e[i] <= customersEnergyUBs[i]);
    c.setName("Eneergy level UB");
    compact_model.model.add(c);
    //clean
    c.end();
  }
}
