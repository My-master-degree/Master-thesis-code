#include "models/vertex.hpp"
#include "models/gvrp_models/cplex/cubic_model/extra_constraint.hpp"
#include "models/gvrp_models/cplex/cubic_model/energy_ub_constraint.hpp"
#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp"
#include "utils/util.hpp"

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace utils;
using namespace models;
using namespace models::gvrp_models::cplex::cubic_model;

Energy_ub_constraint::Energy_ub_constraint (Cubic_model& cubic_model) : Extra_constraint (cubic_model){
}

void Energy_ub_constraint::add () {
  IloConstraint c;
  for (const Vertex& customer : cubic_model.instance.customers){
    c = IloConstraint (cubic_model.e[customer.id] <= calculateCustomerMaxRequiredFuel(cubic_model.instance, *cubic_model.gvrp_afs_tree, customer));
    c.setName("Eneergy level UB");
    cubic_model.model.add(c);
    c.end();
  }
}
