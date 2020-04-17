#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp"
#include "models/gvrp_models/cplex/cubic_model/improved_cubic_model.hpp"
#include "models/gvrp_models/cplex/cubic_model/improved_subcycle_lazy_constraint.hpp"
#include "models/gvrp_models/cplex/cubic_model/lazy_constraint.hpp"
#include "models/gvrp_models/gvrp_instance.hpp"

using namespace models;
using namespace models::gvrp_models::cplex::cubic_model;

Improved_cubic_model::Improved_cubic_model(const Gvrp_instance& gvrp_instance_, unsigned int time_limit_): Cubic_model(gvrp_instance_, time_limit_) {}

Lazy_constraint* Improved_cubic_model::separation_algorithm(){
  return new Improved_subcycle_lazy_constraint(*this);
}
