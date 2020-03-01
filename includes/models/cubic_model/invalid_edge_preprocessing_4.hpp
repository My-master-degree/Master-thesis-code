#ifndef INVALID_EDGE_PREPROCESSING_4_HPP_
#define INVALID_EDGE_PREPROCESSING_4_HPP_

#include "models/cubic_model/cubic_model.hpp"
#include "models/cubic_model/preprocessing_cubic_model.hpp"

namespace models {
  namespace cubic_model {
    class Invalid_edge_preprocessing_4 : public Preprocessing_cubic_model {
      public:
        explicit Invalid_edge_preprocessing_4 (Cubic_model& cubic_model);
        void add ();
    };
  }
}

#endif
