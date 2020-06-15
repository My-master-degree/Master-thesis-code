#ifndef HEURISTIC_CALLBACK_MATHEUS_MODEL_2_CPLEX_HPP_
#define HEURISTIC_CALLBACK_MATHEUS_MODEL_2_CPLEX_HPP_

#include "models/gvrp_models/cplex/matheus_model_2/matheus_model_2.hpp" 

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models;

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace matheus_model_2 {
        class Heuristic_callback : public IloCplex::HeuristicCallbackI {
          protected:
            Matheus_model_2& matheus_model_2;
            const double EPS;
          public:
            explicit Heuristic_callback (Matheus_model_2& matheus_model_2);
        };
      } 
    } 
  } 
}
#endif
