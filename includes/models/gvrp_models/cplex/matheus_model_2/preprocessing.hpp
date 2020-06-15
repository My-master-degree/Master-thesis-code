#ifndef PREPROCESSING_MATHEUS_MODEL_2_CPLEX_HPP_
#define PREPROCESSING_MATHEUS_MODEL_2_CPLEX_HPP_

#include "models/gvrp_models/cplex/matheus_model_2/matheus_model_2.hpp" 

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace matheus_model_2 {
        class Matheus_model_2;
        class Preprocessing {
          protected:
            Matheus_model_2& matheus_model_2;
          public:
            explicit Preprocessing (Matheus_model_2& matheus_model_2);
            virtual void add () = 0;
        };
      }
    }
  }
}

#endif
