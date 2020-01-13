#include "utils/cplex/compact_model.hpp"
#include "utils/cplex/extra_constraint_compact_model.hpp"
#include "utils/cplex/energy_lb_constraint.hpp"
#include "models/distances_enum.hpp"

#include <float.h>
#include <ilcplex/ilocplex.h>
#include <string>

using namespace std;
using namespace utils::cplex;
using namespace models;

Energy_lb_constraint::Energy_lb_constraint (Compact_model& compact_model) : Extra_constraint_compact_model (compact_model) {
  if (compact_model.gvrp_instance.distances_enum != METRIC)
    throw string("The constraint 'Energy LB' only applies for metric instances");
}

void Energy_lb_constraint::add () {
  if (compact_model.gvrp_instance.distances_enum != METRIC)
    throw string("The constraint 'Energy LB' only applies for metric instances");
  IloConstraint c;
  IloExpr rhs (compact_model.env);
  for (Vertex customer : compact_model.gvrp_instance.customers){
    int i = customer.id;
    //min_{f \in F} c_{fi} = minAfsFuel 
    double minAfsFuel = DBL_MAX;
    for (Vertex afs : compact_model.gvrp_instance.afss)
      minAfsFuel = min(minAfsFuel, compact_model.gvrp_instance.distances[i][afs.id]);    
    minAfsFuel *= compact_model.gvrp_instance.vehicleFuelConsumptionRate;    
    //e_i >= min (c_{i,0}, minAfsFuel)
    c = IloConstraint(compact_model.e[i] >= min (compact_model.gvrp_instance.distances[i][0] * compact_model.gvrp_instance.vehicleFuelConsumptionRate, minAfsFuel));
    c.setName("Energy LB");
    compact_model.model.add(c);
    //clean
    rhs.end();
    c.end();
    rhs = IloExpr (compact_model.env);
  }
}
