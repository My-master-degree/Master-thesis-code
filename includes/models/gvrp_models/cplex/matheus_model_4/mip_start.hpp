#ifndef MIP_START_MATHEUS_MODEL_4_CPLEX_HPP_
#define MIP_START_MATHEUS_MODEL_4_CPLEX_HPP_

#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/gvrp_models/gvrp_solution.hpp"
#include "models/gvrp_models/cplex/matheus_model_4/matheus_model_4.hpp"

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace matheus_model_4 {
        class Mip_start : public Matheus_model_4 {
          public:
            explicit Mip_start (const Gvrp_instance& gvrp_instance, unsigned int time_limit, const Gvrp_solution& gvrp_solution); 
            const Gvrp_solution& gvrp_solution;
          protected:
            void extraStepsAfterModelCreation();
        };
      }
    }
  }
}

#endif
