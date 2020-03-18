#include "models/cubic_model/cubic_model.hpp"
#include "models/cubic_model/extra_constraint_cubic_model.hpp"
#include "models/cubic_model/routes_order_constraint.hpp"

#include <ilcplex/ilocplex.h>

using namespace models::cubic_model;

Routes_order_constraint::Routes_order_constraint (Cubic_model& cubic_model) : Extra_constraint_cubic_model (cubic_model) {}

void Routes_order_constraint::add() {
  IloExpr lhs (cubic_model.env), 
          rhs (cubic_model.env);
  IloConstraint c;
  for (int k = 1; k < cubic_model.gvrp_instance.nRoutes; k++) {
    c = IloConstraint (cubic_model.x[k][0][0] == 0);
    c.setName("Depot invalid edge");
    cubic_model.model.add(c);
    c.end();
    for (pair<int, Vertex> p : cubic_model.all) {
      //\sum_{(0, j) \in E} x_{0i}^k
      lhs += cubic_model.x[k][0][p.first];
      //\sum_{(0, j) \in E} x_{0i}^{k-1}
      rhs += cubic_model.x[k][k - 1][p.first];
    }
    //lhs \geqslant rhs
    c = IloConstraint (lhs >= rhs);
    c.setName("Route order");
    cubic_model.model.add(c);
    //clean
    lhs.end();
    rhs.end();
    c.end();
    lhs = IloExpr (cubic_model.env);
    rhs = IloExpr (cubic_model.env);
  }
}
