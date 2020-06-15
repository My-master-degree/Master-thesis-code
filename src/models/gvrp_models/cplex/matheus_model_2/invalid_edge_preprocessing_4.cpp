#include "models/gvrp_models/gvrp_feasible_solution_heuristic.hpp"
#include "models/gvrp_models/cplex/matheus_model_2/matheus_model_2.hpp"
#include "models/gvrp_models/cplex/matheus_model_2/preprocessing.hpp"
#include "models/gvrp_models/cplex/matheus_model_2/invalid_edge_preprocessing_4.hpp"
#include "utils/util.hpp"

#include <list>

using namespace std;
using namespace utils;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex::matheus_model_2;

Invalid_edge_preprocessing_4::Invalid_edge_preprocessing_4 (Matheus_model_2& matheus_model_2) : Preprocessing (matheus_model_2) {
  if (matheus_model_2.instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 4' only applies for metric instances");
}

void Invalid_edge_preprocessing_4::add () {
  list<pair<int, int>> edges = get_invalid_edges_4 (matheus_model_2.instance, *matheus_model_2.gvrp_afs_tree);
  for (auto const& [i, j] : edges) {
    auto f1 = matheus_model_2.afssF0Indexes.find(i),
         f2 = matheus_model_2.afssF0Indexes.find(j);
    //first is afs and second is customer
    if (f1 != matheus_model_2.afssF0Indexes.end()) 
      for (size_t k = 0; k < matheus_model_2.c0.size(); ++k)
        matheus_model_2.model.add(matheus_model_2.y[matheus_model_2.customersC0Indexes[j]][f1->second][k] == 0);
    //first is customer and second is afs 
    else if (f2 != matheus_model_2.afssF0Indexes.end()) 
      for (size_t k = 0; k < matheus_model_2.c0.size(); ++k)
        matheus_model_2.model.add(matheus_model_2.y[matheus_model_2.customersC0Indexes[i]][f2->second][k] == 0);
  }
}
