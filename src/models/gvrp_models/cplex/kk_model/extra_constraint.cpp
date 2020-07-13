#include "models/gvrp_models/cplex/kk_model/kk_model.hpp"
#include "models/gvrp_models/cplex/kk_model/extra_constraint.hpp"

using namespace models::gvrp_models::cplex::kk_model;

Extra_constraint::Extra_constraint (KK_model& kk_model_) : kk_model (kk_model_) {
}

Extra_constraint::~Extra_constraint() {}
