#include "utils/cplex/no_consecutive_afs_visit_preprocessing.hpp"
#include "utils/cplex/preprocessing_compact_model.hpp"

#include <ilcplex/ilocplex.h>

using namespace utils::cplex;

No_consecutive_afs_visit_preprocessing::No_consecutive_afs_visit_preprocessing (Compact_model& compact_model) : Preprocessing_compact_model (compact_model){
}

void No_consecutive_afs_visit_preprocessing::add () {
  for (Vertex v_r : compact_model.gvrp_instance.afss)
    for (Vertex v_f : compact_model.gvrp_instance.afss)
      for (unsigned int k = 0; k < compact_model.gvrp_instance.customers.size(); k++) 
        compact_model.model.add(compact_model.x[k][v_r.id][v_f.id] == 0);
}
