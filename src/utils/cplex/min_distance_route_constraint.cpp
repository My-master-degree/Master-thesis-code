#include "utils/cplex/compact_model.hpp"
#include "utils/cplex/extra_constraint_compact_model.hpp"
#include "utils/cplex/min_distance_route_constraint.hpp"
#include "models/distances_enum.hpp"

#include <ilcplex/ilocplex.h>
#include <string>

using namespace std;
using namespace utils::cplex;
using namespace models;

Min_distance_route_constraint::Min_distance_route_constraint (Compact_model& compact_model, double lambda_) : Extra_constraint_compact_model (compact_model), lambda (lambda_) {
  if (compact_model.gvrp_instance.distances_enum != METRIC)
    throw string("The constraint 'Min distance route' only applies for metric instances");
}

void Min_distance_route_constraint::add (){
  if (compact_model.gvrp_instance.distances_enum != METRIC)
    throw string("The constraint 'Min distance route' only applies for metric instances");
  IloConstraint c;
  IloExpr lhs (compact_model.env),
          rhs (compact_model.env);
  for (unsigned int k = 0; k < compact_model.gvrp_instance.customers.size(); k++){
    //sum_{(i, j) \in E} x_{ij}^k c_{ij} = lhs
    for (pair<int, Vertex> p : compact_model.all) {
      int i = p.first;
      for (pair<int, Vertex> p1 : compact_model.all) {
        int j = p1.first;
        lhs += compact_model.gvrp_instance.distances[i][j] * compact_model.x[k][i][j] * compact_model.gvrp_instance.vehicleFuelConsumptionRate;
      }
    }
    //sum_{(f, i) \in E : f \in F and i \in V} x_{fi}^k = rhs
    for (Vertex afs : compact_model.gvrp_instance.afss)
      for (pair<int, Vertex> p : compact_model.all)
        rhs += compact_model.x[k][afs.id][p.first];
    //rhs * \beta/2 + \lambda
    rhs = (rhs * compact_model.gvrp_instance.vehicleFuelCapacity / 2) + lambda;
    //lhs > rhs
    c = IloConstraint (lhs >= rhs);
    c.setName("route min distance");
    compact_model.model.add(c);
    //clean
    lhs.end();
    rhs.end();
    c.end();
    lhs = IloExpr (compact_model.env);
    rhs = IloExpr (compact_model.env);
  }
}
