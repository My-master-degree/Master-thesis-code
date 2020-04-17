#include "models/vertex.hpp"
#include "models/distances_enum.hpp"
#include "utils/util.hpp"
#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp"
#include "models/gvrp_models/cplex/cubic_model/extra_constraint.hpp"
#include "models/gvrp_models/cplex/cubic_model/energy_lb_constraint.hpp"

#include <float.h>
#include <ilcplex/ilocplex.h>
#include <string>

using namespace std;
using namespace models;
using namespace models::gvrp_models::cplex::cubic_model;
using namespace utils;

Energy_lb_constraint::Energy_lb_constraint (Cubic_model& cubic_model) : Extra_constraint (cubic_model) {
  if (cubic_model.instance.distances_enum != METRIC)
    throw string("The constraint 'Energy LB' only applies for metric instances");
}

void Energy_lb_constraint::add () {
  if (cubic_model.instance.distances_enum != METRIC)
    throw string("The constraint 'Energy LB' only applies for metric instances");
  IloConstraint c;
  IloExpr rhs (cubic_model.env);
  for (const Vertex& customer : cubic_model.instance.customers){
    //e_i >= min (c_{i,0}, minAfsFuel)
    c = IloConstraint(cubic_model.e[customer.id] >= calculateCustomerMinRequiredFuel(cubic_model.instance, *cubic_model.gvrp_afs_tree, customer));
    c.setName("Energy LB");
    cubic_model.model.add(c);
    //clean
    rhs.end();
    c.end();
    rhs = IloExpr (cubic_model.env);
  }
}
