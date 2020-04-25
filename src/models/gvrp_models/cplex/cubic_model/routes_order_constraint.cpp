#include "models/vertex.hpp"
#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp"
#include "models/gvrp_models/cplex/cubic_model/extra_constraint.hpp"
#include "models/gvrp_models/cplex/cubic_model/routes_order_constraint.hpp"

#include <ilcplex/ilocplex.h>

using namespace models;
using namespace models::gvrp_models::cplex::cubic_model;

Routes_order_constraint::Routes_order_constraint (Cubic_model& cubic_model) : Extra_constraint (cubic_model) {}

void Routes_order_constraint::add() {
  IloExpr lhs (cubic_model.env), 
          rhs (cubic_model.env);
  IloConstraint c;
  if (cubic_model.instance.customers.size() > 0) {
    for (const pair<int, const Vertex *>& p : cubic_model.all) 
      //\sum_{(0, j) \in E} x_{0i}^k
      lhs += cubic_model.x[0][cubic_model.instance.depot.id][p.first];
    //lhs \geqslant rhs
    c = IloConstraint (lhs == 1);
    c.setName("Route order");
    cubic_model.model.add(c);
    //clean
    lhs.end();
    c.end();
    lhs = IloExpr (cubic_model.env);
  }
  for (int k = 1; k < cubic_model.instance.nRoutes; k++) {
    for (const pair<int, const Vertex *>& p : cubic_model.all) {
      //\sum_{(0, j) \in E} x_{0i}^k
      lhs += cubic_model.x[k][cubic_model.instance.depot.id][p.first];
      //\sum_{(0, j) \in E} x_{0i}^{k-1}
      rhs += cubic_model.x[k - 1][cubic_model.instance.depot.id][p.first];
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