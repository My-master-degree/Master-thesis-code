#ifndef MAX_AFS_VISIT_CONSTRAINT_HPP_
#define MAX_AFS_VISIT_CONSTRAINT_HPP_

#include "utils/cplex/compact_model.hpp"
#include "utils/cplex/extra_constraint_compact_model.hpp"

namespace utils {
  namespace cplex {
    class Max_afs_visit_constraint: public Extra_constraint_compact_model {
      public:
        explicit Max_afs_visit_constraint (Compact_model& compact_model);
        void add ();
    };
  }
}

#endif
