#include "models/vertex.hpp"
#include "models/gvrp_models/cplex/matheus_model/matheus_model.hpp"
#include "models/gvrp_models/cplex/matheus_model/extra_constraint.hpp"
#include "models/gvrp_models/cplex/matheus_model/energy_ub_constraint.hpp"
#include "utils/util.hpp"

#include <float.h>
#include <map>
#include <ilcplex/ilocplex.h>

using namespace std;
using namespace utils;
using namespace models;
using namespace models::gvrp_models::cplex::matheus_model;

Energy_ub_constraint::Energy_ub_constraint (Matheus_model& matheus_model) : Extra_constraint (matheus_model){
}

void Energy_ub_constraint::add () {
  IloConstraint c;
  for (size_t i = 1; i < matheus_model.c0.size(); ++i){
    /*
    c = IloConstraint (matheus_models.e[i - 1] <= calculateCustomerMaxRequiredFuel(matheus_models.instance, *matheus_models.gvrp_afs_tree, *matheus_models.c0[i]));
    c.setName("Energy level UB");
    matheus_models.model.add(c);
    c.end();
    */
  }
}
