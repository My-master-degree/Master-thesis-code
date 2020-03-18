#include "models/cubic_model/cubic_model.hpp"
#include "models/cubic_model/extra_constraint_cubic_model.hpp"
#include "models/cubic_model/energy_lifting_constraint.hpp"

#include <ilcplex/ilocplex.h>

using namespace models::cubic_model;

Energy_lifting_constraint::Energy_lifting_constraint (Cubic_model& cubic_model) : Extra_constraint_cubic_model (cubic_model) {}

void Energy_lifting_constraint::add () {
  IloExpr rhs (cubic_model.env);
  IloConstraint c;
  //e_j \leq e_i - c_{ij} x_{ij}^k + \beta (1 - x_{ij}^k), \forall v_j \in C,\forall v_i \in V, \forall k \in M
  for (int k = 0; k < cubic_model.gvrp_instance.nRoutes; k++) {
    for (Vertex customer : cubic_model.gvrp_instance.customers) {
      int j = customer.id;
      for (pair<int, Vertex> p :cubic_model.all) {
        int i =  p.first;
        rhs = cubic_model.e[i] - cubic_model.x[k][i][j] * cubic_model.gvrp_instance.distances[i][j] * cubic_model.gvrp_instance.vehicleFuelConsumptionRate + cubic_model.gvrp_instance.vehicleFuelCapacity * (1 - cubic_model.x[k][i][j]) - (cubic_model.gvrp_instance.vehicleFuelCapacity - cubic_model.gvrp_instance.distances[j][i]) * cubic_model.x[k][j][i];
        c = IloConstraint (cubic_model.e[j] <= rhs);
        c.setName("updating fuel level with lifting");
        cubic_model.model.add(c);
        rhs.end();
        c.end();
        rhs = IloExpr (cubic_model.env);
      }
    }
  }
}
