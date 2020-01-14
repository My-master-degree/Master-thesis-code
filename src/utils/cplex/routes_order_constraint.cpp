#include "utils/cplex/compact_model.hpp"
#include "utils/cplex/extra_constraint_compact_model.hpp"
#include "utils/cplex/routes_order_constraint.hpp"

#include <ilcplex/ilocplex.h>

using namespace utils::cplex;

Routes_order_constraint::Routes_order_constraint (Compact_model& compact_model) : Extra_constraint_compact_model (compact_model) {}

void Routes_order_constraint::add() {
  IloExpr lhs (compact_model.env), 
          rhs (compact_model.env);
  IloConstraint c;
  for (unsigned int k = 1; k < compact_model.gvrp_instance.customers.size(); k++) {
    c = IloConstraint (compact_model.x[k][0][0] == 0);
    c.setName("Depot invalid edge");
    compact_model.model.add(c);
    c.end();
    for (pair<int, Vertex> p : compact_model.all) {
      //\sum_{(0, j) \in E} x_{0i}^k
      lhs += compact_model.x[k][0][p.first];
      //\sum_{(0, j) \in E} x_{0i}^{k-1}
      rhs += compact_model.x[k][k - 1][p.first];
    }
    //lhs \geqslant rhs
    c = IloConstraint (lhs >= rhs);
    c.setName("Route order");
    compact_model.model.add(c);
    //clean
    lhs.end();
    rhs.end();
    c.end();
    lhs = IloExpr (compact_model.env);
    rhs = IloExpr (compact_model.env);
  }
}
