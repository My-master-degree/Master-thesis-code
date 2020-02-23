#ifndef INVALID_EDGE_PREPROCESSING_4_HPP_
#define INVALID_EDGE_PREPROCESSING_4_HPP_

#include "utils/cplex/compact_model.hpp"
#include "utils/cplex/preprocessing_compact_model.hpp"

namespace utils {
  namespace cplex {
    class Invalid_edge_preprocessing_4 : public Preprocessing_compact_model {
      public:
        explicit Invalid_edge_preprocessing_4 (Compact_model& compact_model);
        void add ();
    };
  }
}

#endif
