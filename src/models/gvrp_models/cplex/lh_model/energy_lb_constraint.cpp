#include "models/vertex.hpp"
#include "models/distances_enum.hpp"
#include "models/gvrp_models/cplex/lh_model/lh_model.hpp"
#include "models/gvrp_models/cplex/lh_model/extra_constraint.hpp"
#include "models/gvrp_models/cplex/lh_model/energy_lb_constraint.hpp"
#include "utils/util.hpp"

#include <float.h>
#include <ilcplex/ilocplex.h>
#include <string>

using namespace std;
using namespace utils;
using namespace models;
using namespace models::gvrp_models::cplex::lh_model;

Energy_lb_constraint::Energy_lb_constraint (LH_model& lh_model) : Extra_constraint (lh_model) {
  if (lh_model.instance.distances_enum != METRIC)
    throw string("The constraint 'Energy LB' only applies for metric instances");
}

void Energy_lb_constraint::add () {
  IloConstraint c;
  IloExpr rhs (lh_model.env);
  for (size_t i = 1; i < lh_model.c0.size(); ++i){
    //e_i >= min (c_{i,0}, minAfsFuel)
    c = IloConstraint(lh_model.e[i - 1] >= calculateCustomerMinRequiredFuel(lh_model.instance, *lh_model.gvrp_afs_tree, *lh_model.c0[i]));
    c.setName("Energy LB");
    lh_model.model.add(c);
    //clean
    rhs.end();
    c.end();
    rhs = IloExpr (lh_model.env);
  }
}
