#include "models/cubic_model/cubic_model.hpp"
#include "models/cubic_model/extra_constraint_cubic_model.hpp"
#include "models/cubic_model/custom_test_constraint.hpp"

#include <ilcplex/ilocplex.h>
#include <string>

using namespace models::cubic_model;

Custom_test_constraint::Custom_test_constraint (Cubic_model& cubic_model, Gvrp_solution& gvrp_solution_) : Extra_constraint_cubic_model (cubic_model), gvrp_solution (gvrp_solution_) {}

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
      fuel = cubic_model.gvrp_instance.vehicleFuelCapacity;
      for (; node != route.end(); i = node->id, node++) {
        //update fuel
        if (cubic_model.customers.count(node->id)) {
          fuel -= cubic_model.gvrp_instance.distances[i][node->id] * cubic_model.gvrp_instance.vehicleFuelConsumptionRate;
          cubic_model.model.add(cubic_model.e[node->id] == fuel);
        } else
          fuel = cubic_model.gvrp_instance.vehicleFuelCapacity;
        cubic_model.model.add(cubic_model.x[k][i][node->id] == 1);
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
