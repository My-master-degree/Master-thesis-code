#ifndef GREEDY_LP_HEURISTIC_MATHEUS_MODEL_3_CPLEX_HPP_
#define GREEDY_LP_HEURISTIC_MATHEUS_MODEL_3_CPLEX_HPP_

#include "models/gvrp_models/cplex/matheus_model_3/matheus_model_3.hpp" 
#include "models/gvrp_models/cplex/matheus_model_3/heuristic_callback.hpp" 

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models;

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace matheus_model_3 {
        class Greedy_lp_heuristic : public Heuristic_callback {
          public:
            Greedy_lp_heuristic (Matheus_model_3& matheus_model_3);
            [[nodiscard]] IloCplex::CallbackI* duplicateCallback() const override;
            void main() override;
        };
      } 
    } 
  } 
}
#endif
