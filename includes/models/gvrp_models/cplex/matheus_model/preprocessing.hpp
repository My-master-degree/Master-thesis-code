#ifndef PREPROCESSING_MATHEUS_MODEL_CPLEX_HPP_
#define PREPROCESSING_MATHEUS_MODEL_CPLEX_HPP_

#include "models/gvrp_models/cplex/matheus_model/matheus_model.hpp" 

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace matheus_model {
        class Matheus_model;
        class Preprocessing {
          protected:
            Matheus_model& matheus_model;
          public:
            explicit Preprocessing (Matheus_model& matheus_model);
            virtual ~Preprocessing ();
            virtual void add () = 0;
        };
      }
    }
  }
}

#endif
