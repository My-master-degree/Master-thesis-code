#ifndef MIP_START_CUBIC_MODEL_HPP_
#define MIP_START_CUBIC_MODEL_HPP_

#include "models/gvrp_instance.hpp"
#include "models/gvrp_solution.hpp"
#include "models/cubic_model/cubic_model.hpp"

namespace models {
  namespace cubic_model {
    class Mip_start_cubic_model : public Cubic_model {
      public:
        explicit Mip_start_cubic_model (Gvrp_instance& gvrp_instance, unsigned int time_limit, Gvrp_solution& gvrp_solution); 
        Gvrp_solution& gvrp_solution;
      protected:
        void extraStepsAfterModelCreation();
    };
  }
}

#endif
