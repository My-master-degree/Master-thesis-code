#include "utils/cplex/compact_model.hpp"
#include "utils/cplex/extra_constraint_compact_model.hpp"
#include "utils/cplex/energy_ub_constraint.hpp"

#include <float.h>
#include <ilcplex/ilocplex.h>

using namespace utils::cplex;

Energy_ub_constraint::Energy_ub_constraint (Compact_model& compact_model) : Extra_constraint_compact_model (compact_model){
}

void Energy_ub_constraint::add () {
  IloConstraint c;
  for (Vertex customer : compact_model.gvrp_instance.customers){
    int i = customer.id;
    //min_{(j, i) \in E} c_{ji} = minFuel
    double minFuel = DBL_MAX;
    for (pair <int, Vertex> p : compact_model.all) {
      int j = p.first;
      if (i != j)
        minFuel = min (minFuel, compact_model.gvrp_instance.distances[i][j]);
    }
    minFuel *= compact_model.gvrp_instance.vehicleFuelConsumptionRate;
    //e_i \leqslant minFuel
    c = IloConstraint (compact_model.e[i] <= minFuel);
    c.setName("Eneergy level UB");
    compact_model.model.add(c);
    //clean
    c.end();
  }
}
