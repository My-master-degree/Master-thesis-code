#ifndef ROUTES_ORDER_CONSTRAINT_HPP_
#define ROUTES_ORDER_CONSTRAINT_HPP_

#include "models/cubic_model/cubic_model.hpp"
#include "models/cubic_model/extra_constraint_cubic_model.hpp"

namespace models {
  namespace cubic_model {
    class Routes_order_constraint : public Extra_constraint_cubic_model {
      public:
        explicit Routes_order_constraint (Cubic_model& cubic_model);
        void add ();
    };
  }
}

#endif