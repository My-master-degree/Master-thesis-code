#include "models/vertex.hpp"
#include "models/gvrp_models/cplex/matheus_model_2/matheus_model_2.hpp"
#include "models/gvrp_models/cplex/matheus_model_2/extra_constraint.hpp"
#include "models/gvrp_models/cplex/matheus_model_2/energy_ub_constraint.hpp"
#include "utils/util.hpp"

#include <float.h>
#include <map>
#include <ilcplex/ilocplex.h>

using namespace std;
using namespace utils;
using namespace models;
using namespace models::gvrp_models::cplex::matheus_model_2;

Energy_ub_constraint::Energy_ub_constraint (Matheus_model_2& matheus_model_2) : Extra_constraint (matheus_model_2){
}

void Energy_ub_constraint::add () {
  IloConstraint c;
  for (size_t i = 1; i < matheus_model_2.c0.size(); ++i){
    /*
    c = IloConstraint (matheus_model_2s.e[i - 1] <= calculateCustomerMaxRequiredFuel(matheus_model_2s.instance, *matheus_model_2s.gvrp_afs_tree, *matheus_model_2s.c0[i]));
    c.setName("Energy level UB");
    matheus_model_2s.model.add(c);
    c.end();
    */
  }
}
