#ifndef ENERGY_LIFTING_CONSTRAINT_HPP_
#define ENERGY_LIFTING_CONSTRAINT_HPP_

#include "utils/cplex/compact_model.hpp"
#include "utils/cplex/extra_constraint_compact_model.hpp"

namespace utils {
  namespace cplex {
    class Energy_lifting_constraint : public Extra_constraint_compact_model {
      public:
        explicit Energy_lifting_constraint (Compact_model& compact_model);
        void add ();
    };
  }
}

#endif
