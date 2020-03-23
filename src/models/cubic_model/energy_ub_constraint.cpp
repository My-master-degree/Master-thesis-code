#include "models/cubic_model/cubic_model.hpp"
#include "models/cubic_model/extra_constraint_cubic_model.hpp"
#include "models/cubic_model/energy_ub_constraint.hpp"
#include "utils/util.hpp"

#include <float.h>
#include <map>
#include <ilcplex/ilocplex.h>

using namespace std;
using namespace utils;
using namespace models::cubic_model;

Energy_ub_constraint::Energy_ub_constraint (Cubic_model& cubic_model) : Extra_constraint_cubic_model (cubic_model){
}

void Energy_ub_constraint::add () {
  map<int, double> customersEnergyUBs = calculateCustomersEnergyUB(cubic_model);
  IloConstraint c;
  for (Vertex customer : cubic_model.instance.customers){
    int i = customer.id;
    //e_i \leqslant minEdge
    c = IloConstraint (cubic_model.e[i] <= customersEnergyUBs[i]);
    c.setName("Eneergy level UB");
    cubic_model.model.add(c);
    //clean
    c.end();
  }
}
