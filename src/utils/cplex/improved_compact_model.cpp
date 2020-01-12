#include "utils/cplex/improved_compact_model.hpp"
#include "utils/cplex/compact_model.hpp"
#include "utils/cplex/improved_subcycle_lazy_constraint_compact_model.hpp"
#include "models/gvrp_instance.hpp"
#include "utils/cplex/lazy_constraint_compact_model.hpp"

using namespace models;
using namespace utils::cplex;

Improved_compact_model::Improved_compact_model(Gvrp_instance& gvrp_instance_, unsigned int time_limit_): Compact_model(gvrp_instance_, time_limit_) {}

Lazy_constraint_compact_model* Improved_compact_model::separation_algorithm(){
  return new Improved_subcycle_lazy_constraint_compact_model(*this);
}
