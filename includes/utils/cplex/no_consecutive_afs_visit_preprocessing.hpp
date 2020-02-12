#ifndef NO_CONSECUTIVE_AFS_VISIT_PREPROCESSING_HPP_
#define NO_CONSECUTIVE_AFS_VISIT_PREPROCESSING_HPP_ 

#include "utils/cplex/compact_model.hpp"
#include "utils/cplex/preprocessing_compact_model.hpp"

namespace utils {
  namespace cplex {
    class No_consecutive_afs_visit_preprocessing: public Preprocessing_compact_model {
      public:
        explicit No_consecutive_afs_visit_preprocessing (Compact_model& compact_model);
        void add ();
    };
  }
}

#endif
