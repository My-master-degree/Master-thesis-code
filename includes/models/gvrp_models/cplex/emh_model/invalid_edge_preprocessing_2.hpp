#ifndef INVALID_EDGE_PREPROCESSING_2_EMH_MODEL_CPLEX_HPP_
#define INVALID_EDGE_PREPROCESSING_2_EMH_MODEL_CPLEX_HPP_

#include "models/gvrp_models/cplex/emh_model/emh_model.hpp"
#include "models/gvrp_models/cplex/emh_model/preprocessing.hpp"

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace emh_model {
        class Invalid_edge_preprocessing_2 : public Preprocessing {
          public:
            explicit Invalid_edge_preprocessing_2 (EMH_model& emh_model);
            void add ();
        };
      }
    }
  }
}

#endif
