#ifndef IMPROVED_COMPACT_MODEL_HPP_
#define IMPROVED_COMPACT_MODEL_HPP_

#include "models/gvrp_instance.hpp"
#include "utils/cplex/compact_model.hpp"

using namespace models;

namespace utils {
  namespace cplex {
    class Improved_compact_model : public Compact_model {
      public:
        explicit Improved_compact_model(Gvrp_instance& gvrp_instance, unsigned int time_limit); 
      private:
        Lazy_constraint_compact_model* separation_algorithm();
    };
  } 
}
#endif
