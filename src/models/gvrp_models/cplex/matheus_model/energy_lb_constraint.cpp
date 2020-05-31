#include "models/vertex.hpp"
#include "models/distances_enum.hpp"
#include "models/gvrp_models/cplex/matheus_model/matheus_model.hpp"
#include "models/gvrp_models/cplex/matheus_model/extra_constraint.hpp"
#include "models/gvrp_models/cplex/matheus_model/energy_lb_constraint.hpp"
#include "utils/util.hpp"

#include <float.h>
#include <ilcplex/ilocplex.h>
#include <string>

using namespace std;
using namespace utils;
using namespace models;
using namespace models::gvrp_models::cplex::matheus_model;

Energy_lb_constraint::Energy_lb_constraint (Matheus_model& matheus_model) : Extra_constraint (matheus_model) {
  if (matheus_model.instance.distances_enum != METRIC)
    throw string("The constraint 'Energy LB' only applies for metric instances");
}

void Energy_lb_constraint::add () {
  IloConstraint c;
  IloExpr rhs (matheus_model.env);
  for (size_t i = 1; i < matheus_model.c0.size(); ++i){
    /*
    //e_i >= min (c_{i,0}, minAfsFuel)
    c = IloConstraint(matheus_models.e[i - 1] >= calculateCustomerMinRequiredFuel(matheus_models.instance, *matheus_models.gvrp_afs_tree, *matheus_models.c0[i]));
    c.setName("Energy LB");
    matheus_models.model.add(c);
    //clean
    rhs.end();
    c.end();
    rhs = IloExpr (matheus_models.env);
    */
  }
}
