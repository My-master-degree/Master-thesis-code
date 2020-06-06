#include "models/vertex.hpp"
#include "models/cplex/mip_depth.hpp"
#include "models/gvrp_models/cplex/matheus_model/matheus_model.hpp"
#include "models/gvrp_models/cplex/matheus_model/greedy_lp_heuristic.hpp"

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models::gvrp_models::cplex::matheus_model;

Heuristic_callback::Heuristic_callback (Matheus_model& matheus_model_) : IloCplex::HeuristicCallbackI (matheus_model_.env), matheus_model(matheus_model_), EPS(1e-3) {}
