#include "models/cubic_model/cubic_model.hpp"
#include "models/cubic_model/extra_constraint_cubic_model.hpp"
#include "models/cubic_model/min_distance_route_constraint.hpp"
#include "models/distances_enum.hpp"

#include <ilcplex/ilocplex.h>
#include <algorithm>
#include <string>

using namespace std;
using namespace models::cubic_model;
using namespace models;

Min_distance_route_constraint::Min_distance_route_constraint (Cubic_model& cubic_model, double lambda_) : Extra_constraint_cubic_model (cubic_model), lambda (lambda_) {
  if (cubic_model.gvrp_instance.distances_enum != METRIC)
    throw string("The constraint 'Min distance route' only applies for metric instances");
}

void Min_distance_route_constraint::add (){
  if (cubic_model.gvrp_instance.distances_enum != METRIC)
    throw string("The constraint 'Min distance route' only applies for metric instances");
  IloConstraint c;
  IloExpr lhs (cubic_model.env),
          rhs (cubic_model.env);
  for (int k = 0; k < cubic_model.gvrp_instance.nRoutes; k++)
    for (const pair<int, Vertex>& p : cubic_model.all) 
      for (const pair<int, Vertex>& p1 : cubic_model.all) {
        //sum_{(i, j) \in E} x_{ij}^k c_{ij} = lhs
        for (const pair<int, Vertex>& p2 : cubic_model.all) {
          int i = p2.first;
          for (const pair<int, Vertex>& p3 : cubic_model.all) {
            int j = p3.first;
            lhs += cubic_model.gvrp_instance.distances[i][j] * cubic_model.x[k][i][j] * cubic_model.gvrp_instance.vehicleFuelConsumptionRate;
          }
        }
        //sum_{(f, i) \in E : f \in F and i \in V} x_{fi}^k = rhs
        for (const Vertex& afs : cubic_model.gvrp_instance.afss)
          for (const pair<int, Vertex>& p2 : cubic_model.all)
            rhs += cubic_model.x[k][afs.id][p2.first];
        //rhs * \beta/2 + \lambda
        rhs = (rhs * max(lambda, cubic_model.gvrp_instance.vehicleFuelCapacity / 2)) + lambda * cubic_model.x[k][p.first][p1.first];
        //lhs > rhs
        c = IloConstraint (lhs >= rhs);
        c.setName("route min distance");
        cubic_model.model.add(c);
        //clean
        lhs.end();
        rhs.end();
        c.end();
        lhs = IloExpr (cubic_model.env);
        rhs = IloExpr (cubic_model.env);
      }
}
