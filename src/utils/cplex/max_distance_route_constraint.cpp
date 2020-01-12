#include "utils/cplex/compact_model.hpp"
#include "utils/cplex/extra_constraint_compact_model.hpp"
#include "utils/cplex/max_distance_route_costraint.hpp"

#include <ilcplex/ilocplex.h>

using namespace utils::cplex;

Max_distance_route_constraint::Max_distance_route_constraint (Compact_model& compact_model) : Extra_constraint_compact_model (compact_model) {}

void Max_distance_route_constraint::add (){
  IloExpr lhs (env),
          rhs (env);
  IloConstraint c;
  //\sum_{(i, j) \in E} c_{ij} . C . x_{ij}^k = lhs
  for (unsigned int k = 0; k < compact_model.gvrp_instance.customers.size(); k++){
    for (pair<int, Vertex> p : compact_model.all)
      for (pair<int, Vertex> p1 : compact_model.all)
        lhs += compact_model.c[k][p.first][p1.first];
    //(\sum_{f \in F} \sum_{j \in V} x_{fj}^k) + 1 = rhs
    for (Vertex afs : compact_model.gvrp_instance.afss)
      for (pair<int, Vertex> p : compact_model.all)
        rhs += compact_model.x[k][afs.id][p.first];
    //(rhs + 1) * \beta
    rhs = (rhs +  1) * compact_model.gvrp_instance.vehicleFuelCapacity;
    c = lhs <= rhs;
    c.setName("max_distance_route");
    //add constraint
    compact_model.model.add(c);
    cout<<c<<<endl;
    //clean
    lhs.end();
    rhs.end();
    c.end();
    lhs = IloExpr (compact_model.env) ;
    rhs = IloExpr (compact_model.env) ;
    c = IloConstraint (compact_model.env) ;
  }
  
  

}
