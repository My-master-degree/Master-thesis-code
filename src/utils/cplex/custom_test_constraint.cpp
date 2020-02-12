#include "utils/cplex/compact_model.hpp"
#include "utils/cplex/extra_constraint_compact_model.hpp"
#include "utils/cplex/custom_test_constraint.hpp"

#include <ilcplex/ilocplex.h>
#include <string>

using namespace utils::cplex;

Custom_test_constraint::Custom_test_constraint (Compact_model& compact_model, Gvrp_solution& gvrp_solution_) : Extra_constraint_compact_model (compact_model), gvrp_solution (gvrp_solution_) {}

void Custom_test_constraint::add () {
  try {
    //get values
    int k = 0, 
        i;
    double fuel;
    for (list<Vertex> route : gvrp_solution.routes) {
      auto node = route.begin();
      i = node->id;
      node++;
      fuel = compact_model.gvrp_instance.vehicleFuelCapacity;
      for (; node != route.end(); i = node->id, node++) {
        //update fuel
        if (compact_model.customers.count(node->id)) {
          fuel -= compact_model.gvrp_instance.distances[i][node->id] * compact_model.gvrp_instance.vehicleFuelConsumptionRate;
          compact_model.model.add(compact_model.e[node->id] == fuel);
        } else
          fuel = compact_model.gvrp_instance.vehicleFuelCapacity;
        compact_model.model.add(compact_model.x[k][i][node->id] == 1);
      }
      k++;
    }
  } catch (IloException& e) {
    throw e;
  } catch (string& s) {
    throw s;
  } catch (...) {
    throw string("Error in setting custom solution");
  }
}
