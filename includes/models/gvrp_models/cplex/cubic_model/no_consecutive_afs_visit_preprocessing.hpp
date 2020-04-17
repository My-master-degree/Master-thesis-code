#ifndef NO_CONSECUTIVE_AFS_VISIT_PREPROCESSING_CUBIC_MODEL_CPLEX_HPP_
#define NO_CONSECUTIVE_AFS_VISIT_PREPROCESSING_CUBIC_MODEL_CPLEX_HPP_ 

#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp"
#include "models/gvrp_models/cplex/cubic_model/preprocessing.hpp"

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace cubic_model {
        class No_consecutive_afs_visit_preprocessing: public Preprocessing {
          public:
            explicit No_consecutive_afs_visit_preprocessing (Cubic_model& cubic_model);
            void add ();
        };
      }
    }
  }
}

#endif
