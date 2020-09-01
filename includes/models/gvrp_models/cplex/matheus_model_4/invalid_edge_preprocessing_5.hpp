#ifndef INVALID_EDGE_PREPROCESSING_5_MATHEUS_MODEL_4_CPLEX_HPP_
#define INVALID_EDGE_PREPROCESSING_5_MATHEUS_MODEL_4_CPLEX_HPP_

#include "models/gvrp_models/cplex/matheus_model_4/matheus_model_4.hpp"
#include "models/gvrp_models/cplex/matheus_model_5/preprocessing.hpp"

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace matheus_model_4 {
        class Invalid_edge_preprocessing_5 : public Preprocessing {
          public:
            explicit Invalid_edge_preprocessing_5 (Matheus_model_4& matheus_model_4);
            void add ();
        };
      }
    }
  }
}

#endif
