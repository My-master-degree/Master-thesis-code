#ifndef ENERGY_UB_CONSTRAINT_HPP_
#define ENERGY_UB_CONSTRAINT_HPP_

#include "models/cubic_model/cubic_model.hpp"
#include "models/cubic_model/extra_constraint_cubic_model.hpp"

namespace models {
  namespace cubic_model {
    class Energy_ub_constraint : public Extra_constraint_cubic_model {
      public:
       explicit Energy_ub_constraint (Cubic_model& cubic_model);
       void add ();
    };
  }
}

#endif
