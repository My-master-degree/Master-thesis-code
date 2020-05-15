#include "models/gvrp_models/cplex/lh_model/lh_model.hpp"
#include "models/gvrp_models/cplex/lh_model/extra_constraint.hpp"

using namespace models::gvrp_models::cplex::lh_model;

Extra_constraint::Extra_constraint (LH_model& lh_model_) : lh_model (lh_model_) {
}
