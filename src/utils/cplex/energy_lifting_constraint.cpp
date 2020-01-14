#include "utils/cplex/compact_model.hpp"
#include "utils/cplex/extra_constraint_compact_model.hpp"
#include "utils/cplex/energy_lifting_constraint.hpp"

#include <ilcplex/ilocplex.h>

using namespace utils::cplex;

Energy_lifting_constraint::Energy_lifting_constraint (Compact_model& compact_model) : Extra_constraint_compact_model (compact_model) {}

void Energy_lifting_constraint::add () {
  IloExpr rhs (compact_model.env);
  IloConstraint c;
  //e_j \leq e_i - c_{ij} x_{ij}^k + \beta (1 - x_{ij}^k), \forall v_j \in C,\forall v_i \in V, \forall k \in M
  for (unsigned int k = 0; k < compact_model.gvrp_instance.customers.size(); k++) {
    for (Vertex customer : compact_model.gvrp_instance.customers) {
      int j = customer.id;
      for (pair<int, Vertex> p :compact_model.all) {
        int i =  p.first;
        rhs = compact_model.e[i] - compact_model.x[k][i][j] * compact_model.gvrp_instance.distances[i][j] * compact_model.gvrp_instance.vehicleFuelConsumptionRate + compact_model.gvrp_instance.vehicleFuelCapacity * (1 - compact_model.x[k][i][j]) - (compact_model.gvrp_instance.vehicleFuelCapacity - compact_model.gvrp_instance.distances[j][i]) * compact_model.x[k][j][i];
        c = IloConstraint (compact_model.e[j] <= rhs);
        c.setName("updating fuel level with lifting");
        compact_model.model.add(c);
        rhs.end();
        c.end();
        rhs = IloExpr (compact_model.env);
      }
    }
  }
}
