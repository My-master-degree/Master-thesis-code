#ifndef ENERGY_LIFTING_CONSTRAINT_HPP_
#define ENERGY_LIFTING_CONSTRAINT_HPP_

#include "models/cubic_model/cubic_model.hpp"
#include "models/cubic_model/extra_constraint_cubic_model.hpp"

namespace models {
  namespace cubic_model {
    class Energy_lifting_constraint : public Extra_constraint_cubic_model {
      public:
        explicit Energy_lifting_constraint (Cubic_model& cubic_model);
        void add ();
    };
  }
}

#endif
