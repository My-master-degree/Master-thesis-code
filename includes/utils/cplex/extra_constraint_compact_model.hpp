#ifndef EXTRA_CONSTRAINT_COMPACT_MODEL_HPP_
#define EXTRA_CONSTRAINT_COMPACT_MODEL_HPP_

#include "utils/cplex/compact_model.hpp"

namespace utils {
  namespace cplex {
    class Compact_model;
    class Extra_constraint_compact_model {
      protected:
        Compact_model& compact_model;
      public:
        explicit Extra_constraint_compact_model (Compact_model& compact_model);
        virtual void add() = 0; 
    };
  }
}

#endif
