#include "models/vertex.hpp"
#include "models/distances_enum.hpp"
#include "models/gvrp_models/cplex/matheus_model_2/matheus_model_2.hpp"
#include "models/gvrp_models/cplex/matheus_model_2/extra_constraint.hpp"
#include "models/gvrp_models/cplex/matheus_model_2/energy_lb_constraint.hpp"
#include "utils/util.hpp"

#include <float.h>
#include <ilcplex/ilocplex.h>
#include <string>

using namespace std;
using namespace utils;
using namespace models;
using namespace models::gvrp_models::cplex::matheus_model_2;

Energy_lb_constraint::Energy_lb_constraint (Matheus_model_2& matheus_model_2) : Extra_constraint (matheus_model_2) {
  if (matheus_model_2.instance.distances_enum != METRIC)
    throw string("The constraint 'Energy LB' only applies for metric instances");
}

void Energy_lb_constraint::add () {
  IloConstraint c;
  IloExpr rhs (matheus_model_2.env);
  for (int i = 1; i < matheus_model_2.c0.size(); ++i){
    /*
    //e_i >= min (c_{i,0}, minAfsFuel)
    c = IloConstraint(matheus_model_2s.e[i - 1] >= calculateCustomerMinRequiredFuel(matheus_model_2s.instance, *matheus_model_2s.gvrp_afs_tree, *matheus_model_2s.c0[i]));
    c.setName("Energy LB");
    matheus_model_2s.model.add(c);
    //clean
    rhs.end();
    c.end();
    rhs = IloExpr (matheus_model_2s.env);
    */
  }
}
