#include "models/vertex.hpp"
#include "models/gvrp_models/cplex/emh_model/emh_model.hpp"
#include "models/gvrp_models/cplex/emh_model/extra_constraint.hpp"
#include "models/gvrp_models/cplex/emh_model/energy_lifting_constraint.hpp"

#include <ilcplex/ilocplex.h>

using namespace models;
using namespace models::gvrp_models::cplex::emh_model;

Energy_lifting_constraint::Energy_lifting_constraint (EMH_model& emh_model) : Extra_constraint (emh_model) {}

void Energy_lifting_constraint::add () {
  IloExpr rhs (emh_model.env);
  IloConstraint c;
  //e_j \leq e_i - c_{ij} x_{ij}^k + \beta (1 - x_{ij}^k), \forall v_j \in C,\forall v_i \in V, \forall k \in M
  for (const Vertex& customer : emh_model.instance.customers) {
    int j = customer.id;
    for (const pair<int, const Vertex *>& p :emh_model.all) {
      int i =  p.first;
      if (i != j) {
        rhs = emh_model.e[i] - emh_model.x[i][j] * emh_model.instance.distances[i][j] * emh_model.instance.vehicleFuelConsumptionRate + emh_model.instance.vehicleFuelCapacity * (1 - emh_model.x[i][j]) - (emh_model.instance.vehicleFuelCapacity - emh_model.instance.distances[j][i]) * emh_model.x[j][i];
        c = IloConstraint (emh_model.e[j] <= rhs);
        c.setName("updating fuel level with lifting");
        emh_model.model.add(c);
        rhs.end();
        c.end();
        rhs = IloExpr (emh_model.env);
      }
    }
  }
}
