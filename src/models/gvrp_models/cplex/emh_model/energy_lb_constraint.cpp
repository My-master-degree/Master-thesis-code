#include "models/vertex.hpp"
#include "models/distances_enum.hpp"
#include "models/gvrp_models/cplex/emh_model/emh_model.hpp"
#include "models/gvrp_models/cplex/emh_model/extra_constraint.hpp"
#include "models/gvrp_models/cplex/emh_model/energy_lb_constraint.hpp"
#include "utils/util.hpp"

#include <float.h>
#include <ilcplex/ilocplex.h>
#include <string>

using namespace std;
using namespace utils;
using namespace models;
using namespace models::gvrp_models::cplex::emh_model;

Energy_lb_constraint::Energy_lb_constraint (EMH_model& emh_model) : Extra_constraint (emh_model) {
  if (emh_model.instance.distances_enum != METRIC)
    throw string("The constraint 'Energy LB' only applies for metric instances");
}

void Energy_lb_constraint::add () {
  if (emh_model.instance.distances_enum != METRIC)
    throw string("The constraint 'Energy LB' only applies for metric instances");
  IloConstraint c;
  IloExpr rhs (emh_model.env);
  for (const Vertex& customer : emh_model.instance.customers){
    //e_i >= min (c_{i,0}, minAfsFuel)
    c = IloConstraint(emh_model.e[customer.id] >= calculateCustomerMinRequiredFuel(emh_model.instance, *emh_model.gvrp_afs_tree, customer));
    c.setName("Energy LB");
    emh_model.model.add(c);
    //clean
    rhs.end();
    c.end();
    rhs = IloExpr (emh_model.env);
  }
}
