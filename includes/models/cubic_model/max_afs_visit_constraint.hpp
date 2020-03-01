#ifndef MAX_AFS_VISIT_CONSTRAINT_HPP_
#define MAX_AFS_VISIT_CONSTRAINT_HPP_

#include "models/cubic_model/cubic_model.hpp"
#include "models/cubic_model/extra_constraint_cubic_model.hpp"

namespace models {
  namespace cubic_model {
    class Max_afs_visit_constraint: public Extra_constraint_cubic_model {
      public:
        explicit Max_afs_visit_constraint (Cubic_model& cubic_model);
        void add ();
    };
  }
}

#endif
