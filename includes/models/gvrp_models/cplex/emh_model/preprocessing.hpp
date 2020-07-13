#ifndef PREPROCESSING_EMH_MODEL_CPLEX_HPP_
#define PREPROCESSING_EMH_MODEL_CPLEX_HPP_

#include "models/gvrp_models/cplex/emh_model/emh_model.hpp" 

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace emh_model {
        class EMH_model;
        class Preprocessing {
          protected:
            EMH_model& emh_model;
          public:
            explicit Preprocessing (EMH_model& emh_model);
            virtual ~Preprocessing ();
            virtual void add () = 0;
        };
      }
    }
  }
}

#endif
