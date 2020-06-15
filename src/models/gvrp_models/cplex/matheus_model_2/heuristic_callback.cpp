#include "models/vertex.hpp"
#include "models/cplex/mip_depth.hpp"
#include "models/gvrp_models/cplex/matheus_model_2/matheus_model_2.hpp"
#include "models/gvrp_models/cplex/matheus_model_2/greedy_lp_heuristic.hpp"

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models::gvrp_models::cplex::matheus_model_2;

Heuristic_callback::Heuristic_callback (Matheus_model_2& matheus_model_2_) : IloCplex::HeuristicCallbackI (matheus_model_2_.env), matheus_model_2(matheus_model_2_), EPS(1e-3) {}
