#ifndef CUSTOM_TEST_CONSTRAINT_HPP_
#define CUSTOM_TEST_CONSTRAINT_HPP_

#include "models/gvrp_solution.hpp"
#include "utils/cplex/compact_model.hpp"
#include "utils/cplex/extra_constraint_compact_model.hpp"

namespace utils {
  namespace cplex {
    class Custom_test_constraint : public Extra_constraint_compact_model {
      public:
        explicit Custom_test_constraint (Compact_model& compact_model, Gvrp_solution& gvrp_solution);
        void add ();
      private:
        Gvrp_solution& gvrp_solution;
    };
  }
}

#endif
