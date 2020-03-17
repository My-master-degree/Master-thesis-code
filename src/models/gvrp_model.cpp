#include "models/gvrp_instance.hpp"
#include "models/gvrp_model.hpp"

using namespace models;

Gvrp_model::Gvrp_model (Gvrp_instance& gvrp_instance_, unsigned int time_limit_) : gvrp_instance(gvrp_instance_), time_limit(time_limit_), max_num_feasible_integer_sol(210000000), VERBOSE(true), gvrp_solution(nullptr) {}
