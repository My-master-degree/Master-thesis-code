#include "models/gvrp_models/cplex/kk_model/kk_model.hpp"
#include "models/gvrp_models/cplex/kk_model/preprocessing.hpp"

using namespace models::gvrp_models::cplex::kk_model;

Preprocessing::Preprocessing (KK_model& kk_model_) : kk_model(kk_model_) {}
