#ifndef INVALID_EDGE_PREPROCESSING_4_KK_MODEL_CPLEX_HPP_
#define INVALID_EDGE_PREPROCESSING_4_KK_MODEL_CPLEX_HPP_

#include "models/gvrp_models/cplex/kk_model/kk_model.hpp"
#include "models/gvrp_models/cplex/kk_model/preprocessing.hpp"

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace kk_model {
        class Invalid_edge_preprocessing_4 : public Preprocessing {
          public:
            explicit Invalid_edge_preprocessing_4 (KK_model& kk_model);
            void add ();
        };
      }
    }
  }
}

#endif
