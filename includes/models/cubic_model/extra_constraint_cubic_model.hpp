#ifndef EXTRA_CONSTRAINT_CUBIC_MODEL_HPP_
#define EXTRA_CONSTRAINT_CUBIC_MODEL_HPP_

#include "models/cubic_model/cubic_model.hpp"

namespace models {
  namespace cubic_model {
    class Cubic_model;
    class Extra_constraint_cubic_model {
      protected:
        Cubic_model& cubic_model;
      public:
        explicit Extra_constraint_cubic_model (Cubic_model& cubic_model);
        virtual void add() = 0; 
    };
  }
}

#endif
