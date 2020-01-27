#ifndef MIP_START_COMPACT_MODEL_HPP_
#define MIP_START_COMPACT_MODEL_HPP_

#include "models/gvrp_instance.hpp"
#include "models/gvrp_solution.hpp"
#include "utils/cplex/compact_model.hpp"

namespace utils {
  namespace cplex {
    class Mip_start_compact_model : public Compact_model {
      public:
        explicit Mip_start_compact_model (Gvrp_instance& gvrp_instance, unsigned int time_limit, Gvrp_solution& gvrp_solution); 
        Gvrp_solution& gvrp_solution;
      protected:
        void extraStepsAfterModelCreation();
    };
  }
}

#endif
