#ifndef IMPROVED_CUBIC_MODEL_HPP_
#define IMPROVED_CUBIC_MODEL_HPP_

#include "models/gvrp_instance.hpp"
#include "models/cubic_model/cubic_model.hpp"

using namespace models;

namespace models {
  namespace cubic_model {
    class Improved_cubic_model : public Cubic_model {
      public:
        explicit Improved_cubic_model(Gvrp_instance& gvrp_instance, unsigned int time_limit); 
      private:
        Lazy_constraint_cubic_model* separation_algorithm();
    };
  } 
}
#endif
