#include "models/vertex.hpp"
#include "models/gvrp_models/cplex/cubic_model/no_consecutive_afs_visit_preprocessing.hpp"
#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp"

#include <ilcplex/ilocplex.h>

using namespace models;
using namespace models::gvrp_models::cplex::cubic_model;

No_consecutive_afs_visit_preprocessing::No_consecutive_afs_visit_preprocessing (Cubic_model& cubic_model) : Preprocessing (cubic_model){
}

void No_consecutive_afs_visit_preprocessing::add () {
  for (const Vertex& v_r : cubic_model.instance.afss)
    for (const Vertex& v_f : cubic_model.instance.afss)
      for (int k = 0; k < cubic_model.instance.nRoutes; k++) 
        cubic_model.model.add(cubic_model.x[k][v_r.id][v_f.id] == 0);
}
