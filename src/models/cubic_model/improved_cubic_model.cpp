#include "models/cubic_model/improved_cubic_model.hpp"
#include "models/cubic_model/cubic_model.hpp"
#include "models/cubic_model/improved_subcycle_lazy_constraint_cubic_model.hpp"
#include "models/gvrp_instance.hpp"
#include "models/cubic_model/lazy_constraint_cubic_model.hpp"

using namespace models;
using namespace models::cubic_model;

Improved_cubic_model::Improved_cubic_model(Gvrp_instance& gvrp_instance_, unsigned int time_limit_): Cubic_model(gvrp_instance_, time_limit_) {}

Lazy_constraint_cubic_model* Improved_cubic_model::separation_algorithm(){
  return new Improved_subcycle_lazy_constraint_cubic_model(*this);
}
