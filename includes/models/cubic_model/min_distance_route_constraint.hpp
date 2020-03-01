#ifndef MIN_DISTANCE_ROUTE_CONSTRAINT_HPP_
#define MIN_DISTANCE_ROUTE_CONSTRAINT_HPP_

#include "models/cubic_model/cubic_model.hpp"
#include "models/cubic_model/extra_constraint_cubic_model.hpp"

namespace models {
  namespace cubic_model {
    class Min_distance_route_constraint : public Extra_constraint_cubic_model{
      public:
        explicit Min_distance_route_constraint (Cubic_model& cubic_model, double lambda);
        void add ();
      private:
        double lambda;
    };
  }
}

#endif
