#ifndef INVALID_EDGE_PREPROCESSING_2_EMH_MODEL_HPP_
#define INVALID_EDGE_PREPROCESSING_2_EMH_MODEL_HPP_

#include "models/emh_model/emh_model.hpp"
#include "models/emh_model/preprocessing_emh_model.hpp"

namespace models {
  namespace emh_model {
    class Invalid_edge_preprocessing_2 : public Preprocessing_emh_model {
      public:
        explicit Invalid_edge_preprocessing_2 (EMH_model& emh_model);
        void add ();
    };
  }
}

#endif
