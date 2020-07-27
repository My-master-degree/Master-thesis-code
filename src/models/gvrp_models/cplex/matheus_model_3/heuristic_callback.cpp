#include "models/vertex.hpp"
#include "models/cplex/mip_depth.hpp"
#include "models/gvrp_models/cplex/matheus_model_3/matheus_model_3.hpp"
#include "models/gvrp_models/cplex/matheus_model_3/greedy_lp_heuristic.hpp"

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models::gvrp_models::cplex::matheus_model_3;

Heuristic_callback::Heuristic_callback (Matheus_model_3& matheus_model_3_) : IloCplex::HeuristicCallbackI (matheus_model_3_.env), matheus_model_3(matheus_model_3_), EPS(1e-3) {}
