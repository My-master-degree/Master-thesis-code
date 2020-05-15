#include "models/vertex.hpp"
#include "models/gvrp_models/cplex/lh_model/lh_model.hpp"
#include "models/gvrp_models/cplex/lh_model/extra_constraint.hpp"
#include "models/gvrp_models/cplex/lh_model/energy_ub_constraint.hpp"
#include "utils/util.hpp"

#include <float.h>
#include <map>
#include <ilcplex/ilocplex.h>

using namespace std;
using namespace utils;
using namespace models;
using namespace models::gvrp_models::cplex::lh_model;

Energy_ub_constraint::Energy_ub_constraint (LH_model& lh_model) : Extra_constraint (lh_model){
}

void Energy_ub_constraint::add () {
  IloConstraint c;
  for (size_t i = 1; i < lh_model.c0.size(); ++i){
    c = IloConstraint (lh_model.e[i - 1] <= calculateCustomerMaxRequiredFuel(lh_model.instance, *lh_model.gvrp_afs_tree, *lh_model.c0[i]));
    c.setName("Energy level UB");
    lh_model.model.add(c);
    c.end();
  }
}
