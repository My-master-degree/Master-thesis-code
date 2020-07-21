#ifndef PREPROCESSING_MATHEUS_MODEL_3_CPLEX_HPP_
#define PREPROCESSING_MATHEUS_MODEL_3_CPLEX_HPP_

#include "models/gvrp_models/cplex/matheus_model_3/matheus_model_3.hpp" 

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace matheus_model_3 {
        class Matheus_model_3;
        class Preprocessing {
          protected:
            Matheus_model_3& matheus_model_3;
          public:
            explicit Preprocessing (Matheus_model_3& matheus_model_3);
            virtual ~Preprocessing ();
            virtual void add () = 0;
        };
      }
    }
  }
}

#endif
