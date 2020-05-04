#include "models/vertex.hpp"
#include "models/gvrp_models/cplex/kk_model/kk_model.hpp"
#include "models/gvrp_models/cplex/kk_model/extra_constraint.hpp"
#include "models/gvrp_models/cplex/kk_model/energy_ub_constraint.hpp"
#include "utils/util.hpp"

#include <float.h>
#include <map>
#include <ilcplex/ilocplex.h>

using namespace std;
using namespace utils;
using namespace models;
using namespace models::gvrp_models::cplex::kk_model;

Energy_ub_constraint::Energy_ub_constraint (KK_model& kk_model) : Extra_constraint (kk_model){
}

void Energy_ub_constraint::add () {
  IloConstraint c;
  for (size_t i = 1; i < kk_model.c0.size(); ++i){
    c = IloConstraint (kk_model.e[i - 1] <= calculateCustomerMaxRequiredFuel(kk_model.instance, *kk_model.gvrp_afs_tree, *kk_model.c0[i]));
    c.setName("Energy level UB");
    kk_model.model.add(c);
    c.end();
  }
}
