#ifndef INVALID_EDGE_PREPROCESSING_MATHEUS_MODEL_2_CPLEX_HPP_
#define INVALID_EDGE_PREPROCESSING_MATHEUS_MODEL_2_CPLEX_HPP_

#include "models/gvrp_models/cplex/matheus_model_2/matheus_model_2.hpp"
#include "models/gvrp_models/cplex/matheus_model_2/preprocessing.hpp"

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace matheus_model_2 {
        class Invalid_edge_preprocessing : public Preprocessing {
          public:
            explicit Invalid_edge_preprocessing (Matheus_model_2& matheus_model_2);
            void add ();
        };
      }
    }
  }
}

#endif
