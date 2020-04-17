#ifndef IMPROVED_CUBIC_MODEL_CPLEX_HPP_
#define IMPROVED_CUBIC_MODEL_CPLEX_HPP_

#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp"
#include "models/gvrp_models/cplex/cubic_model/lazy_constraint.hpp"

using namespace models;

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace cubic_model {
        class Improved_cubic_model : public Cubic_model {
          public:
            explicit Improved_cubic_model(const Gvrp_instance& gvrp_instance, unsigned int time_limit); 
          private:
            Lazy_constraint* separation_algorithm();
        };
      } 
    } 
  } 
}
#endif
