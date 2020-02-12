#ifndef INVALID_PATHS_CONSTRAINT_HPP_
#define INVALID_PATHS_CONSTRAINT_HPP_ 

#include "utils/cplex/compact_model.hpp"
#include "utils/cplex/extra_constraint_compact_model.hpp"

namespace utils {
  namespace cplex {
    class Invalid_paths_constraint : public Extra_constraint_compact_model {
      public:
        explicit Invalid_paths_constraint (Compact_model& compact_model);
        void add ();
    };
  }
}

#endif
