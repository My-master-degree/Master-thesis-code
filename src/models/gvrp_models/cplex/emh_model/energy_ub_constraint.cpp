#include "models/vertex.hpp"
#include "models/gvrp_models/cplex/emh_model/emh_model.hpp"
#include "models/gvrp_models/cplex/emh_model/extra_constraint.hpp"
#include "models/gvrp_models/cplex/emh_model/energy_ub_constraint.hpp"
#include "utils/util.hpp"

#include <float.h>
#include <map>
#include <ilcplex/ilocplex.h>

using namespace std;
using namespace utils;
using namespace models;
using namespace models::gvrp_models::cplex::emh_model;

Energy_ub_constraint::Energy_ub_constraint (EMH_model& emh_model) : Extra_constraint (emh_model){
}

void Energy_ub_constraint::add () {
  IloConstraint c;
  for (const Vertex& customer : emh_model.instance.customers){
    c = IloConstraint (emh_model.e[customer.id] <= calculateCustomerMaxAllowedFuel(emh_model.instance, *emh_model.gvrp_afs_tree, customer));
    c.setName("Eneergy level UB");
    emh_model.model.add(c);
    c.end();
  }
}
