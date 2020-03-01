#ifndef CUSTOM_TEST_CONSTRAINT_HPP_
#define CUSTOM_TEST_CONSTRAINT_HPP_

#include "models/gvrp_solution.hpp"
#include "models/cubic_model/cubic_model.hpp"
#include "models/cubic_model/extra_constraint_cubic_model.hpp"

namespace models {
  namespace cubic_model {
    class Custom_test_constraint : public Extra_constraint_cubic_model {
      public:
        explicit Custom_test_constraint (Cubic_model& cubic_model, Gvrp_solution& gvrp_solution);
        void add ();
      private:
        Gvrp_solution& gvrp_solution;
    };
  }
}

#endif
