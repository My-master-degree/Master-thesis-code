#ifndef PREPROCESSING_KK_MODEL_CPLEX_HPP_
#define PREPROCESSING_KK_MODEL_CPLEX_HPP_

#include "models/gvrp_models/cplex/kk_model/kk_model.hpp" 

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace kk_model {
        class KK_model;
        class Preprocessing {
          protected:
            KK_model& kk_model;
          public:
            explicit Preprocessing (KK_model& kk_model);
            virtual void add () = 0;
        };
      }
    }
  }
}

#endif
