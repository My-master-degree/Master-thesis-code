#include "models/cubic_model/cubic_model.hpp"
#include "models/cubic_model/extra_constraint_cubic_model.hpp"
#include "models/cubic_model/energy_lb_constraint.hpp"
#include "models/distances_enum.hpp"

#include <float.h>
#include <ilcplex/ilocplex.h>
#include <string>

using namespace std;
using namespace models::cubic_model;
using namespace models;

Energy_lb_constraint::Energy_lb_constraint (Cubic_model& cubic_model) : Extra_constraint_cubic_model (cubic_model) {
  if (cubic_model.gvrp_instance.distances_enum != METRIC)
    throw string("The constraint 'Energy LB' only applies for metric instances");
}

void Energy_lb_constraint::add () {
  if (cubic_model.gvrp_instance.distances_enum != METRIC)
    throw string("The constraint 'Energy LB' only applies for metric instances");
  IloConstraint c;
  IloExpr rhs (cubic_model.env);
  for (Vertex customer : cubic_model.gvrp_instance.customers){
    int i = customer.id;
    //min_{f \in F} c_{fi} = minAfsFuel 
    double minAfsFuel = DBL_MAX;
    for (Vertex afs : cubic_model.gvrp_instance.afss)
      minAfsFuel = min(minAfsFuel, cubic_model.gvrp_instance.distances[i][afs.id]);    
    minAfsFuel *= cubic_model.gvrp_instance.vehicleFuelConsumptionRate;    
    //e_i >= min (c_{i,0}, minAfsFuel)
    c = IloConstraint(cubic_model.e[i] >= min (cubic_model.gvrp_instance.distances[i][0] * cubic_model.gvrp_instance.vehicleFuelConsumptionRate, minAfsFuel));
    c.setName("Energy LB");
    cubic_model.model.add(c);
    //clean
    rhs.end();
    c.end();
    rhs = IloExpr (cubic_model.env);
  }
}
