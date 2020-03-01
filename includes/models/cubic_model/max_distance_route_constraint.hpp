#ifndef MAX_DISTANCE_ROUTE_CONSTRAINT_HPP_
#define MAX_DISTANCE_ROUTE_CONSTRAINT_HPP_ 

#include "models/cubic_model/cubic_model.hpp"
#include "models/cubic_model/extra_constraint_cubic_model.hpp"

namespace models {
  namespace cubic_model {
    class Max_distance_route_constraint : public Extra_constraint_cubic_model {
      public:
        explicit Max_distance_route_constraint (Cubic_model& cubic_model);
        void add ();
    };
  }
}

#endif
