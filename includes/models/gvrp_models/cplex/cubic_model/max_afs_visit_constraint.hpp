#ifndef MAX_AFS_VISIT_CONSTRAINT_CUBIC_MODEL_AFS_HPP_
#define MAX_AFS_VISIT_CONSTRAINT_CUBIC_MODEL_AFS_HPP_

#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp"
#include "models/gvrp_models/cplex/cubic_model/extra_constraint.hpp"

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace cubic_model {
        class Max_afs_visit_constraint: public Extra_constraint {
          public:
            explicit Max_afs_visit_constraint (Cubic_model& cubic_model);
            void add ();
        };
      }
    }
  }
}

#endif
