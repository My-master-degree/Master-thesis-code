#ifndef INVALID_PATHS_CONSTRAINT_HPP_
#define INVALID_PATHS_CONSTRAINT_HPP_ 

#include "models/cubic_model/cubic_model.hpp"
#include "models/cubic_model/extra_constraint_cubic_model.hpp"

namespace models {
  namespace cubic_model {
    class Invalid_paths_constraint : public Extra_constraint_cubic_model {
      public:
        explicit Invalid_paths_constraint (Cubic_model& cubic_model);
        void add ();
    };
  }
}

#endif
