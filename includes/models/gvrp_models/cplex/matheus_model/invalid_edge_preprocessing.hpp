#ifndef INVALID_EDGE_PREPROCESSING_MATHEUS_MODEL_CPLEX_HPP_
#define INVALID_EDGE_PREPROCESSING_MATHEUS_MODEL_CPLEX_HPP_

#include "models/gvrp_models/cplex/matheus_model/matheus_model.hpp"
#include "models/gvrp_models/cplex/matheus_model/preprocessing.hpp"

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace matheus_model {
        class Invalid_edge_preprocessing : public Preprocessing {
          public:
            explicit Invalid_edge_preprocessing (Matheus_model& matheus_model);
            void add ();
        };
      }
    }
  }
}

#endif
