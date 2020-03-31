#ifndef EXTRA_CONSTRAINT_EMH_MODEL_HPP_
#define EXTRA_CONSTRAINT_EMH_MODEL_HPP_

#include "models/emh_model/emh_model.hpp"

namespace models {
  namespace emh_model {
    class EMH_model;
    class Extra_constraint_emh_model {
      protected:
        EMH_model& emh_model;
      public:
        explicit Extra_constraint_emh_model (EMH_model& emh_model);
        virtual void add() = 0; 
    };
  }
}

#endif
