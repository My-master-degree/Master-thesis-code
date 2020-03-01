#ifndef NO_CONSECUTIVE_AFS_VISIT_PREPROCESSING_HPP_
#define NO_CONSECUTIVE_AFS_VISIT_PREPROCESSING_HPP_ 

#include "models/cubic_model/cubic_model.hpp"
#include "models/cubic_model/preprocessing_cubic_model.hpp"

namespace models {
  namespace cubic_model {
    class No_consecutive_afs_visit_preprocessing: public Preprocessing_cubic_model {
      public:
        explicit No_consecutive_afs_visit_preprocessing (Cubic_model& cubic_model);
        void add ();
    };
  }
}

#endif
