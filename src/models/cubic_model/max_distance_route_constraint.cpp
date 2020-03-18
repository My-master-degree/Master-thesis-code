#include "models/cubic_model/cubic_model.hpp"
#include "models/cubic_model/extra_constraint_cubic_model.hpp"
#include "models/cubic_model/max_distance_route_constraint.hpp"

#include <ilcplex/ilocplex.h>

using namespace models::cubic_model;

Max_distance_route_constraint::Max_distance_route_constraint (Cubic_model& cubic_model) : Extra_constraint_cubic_model (cubic_model) {}

void Max_distance_route_constraint::add (){
  IloExpr lhs (cubic_model.env),
          rhs (cubic_model.env);
  IloConstraint c;
  //\sum_{(i, j) \in E} c_{ij} . C . x_{ij}^k = lhs
  for (int k = 0; k < cubic_model.gvrp_instance.nRoutes; k++){
    for (pair<int, Vertex> p : cubic_model.all){
      int i = p.first;
      for (pair<int, Vertex> p1 : cubic_model.all){
        int j = p1.first;
        lhs += cubic_model.x[k][i][j] * cubic_model.gvrp_instance.vehicleFuelConsumptionRate * cubic_model.gvrp_instance.distances[i][j];
      }
    }
    //(\sum_{f \in F} \sum_{j \in V} x_{fj}^k) + 1 = rhs
    for (Vertex afs : cubic_model.gvrp_instance.afss)
      for (pair<int, Vertex> p : cubic_model.all)
        rhs += cubic_model.x[k][afs.id][p.first];
    //(rhs + 1) * \beta
    rhs = (rhs +  1) * cubic_model.gvrp_instance.vehicleFuelCapacity;
    c = IloConstraint (lhs <= rhs);
    c.setName("max_distance_route");
    //add constraint
    cubic_model.model.add(c);
    //clean
    lhs.end();
    rhs.end();
    c.end();
    lhs = IloExpr (cubic_model.env) ;
    rhs = IloExpr (cubic_model.env) ;
  }
  
  

}
