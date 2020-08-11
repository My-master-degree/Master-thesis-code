#include "models/gvrp_models/cplex/matheus_model_2/matheus_model_2.hpp"
#include "models/gvrp_models/gvrp_feasible_solution_heuristic.hpp"
#include "models/gvrp_models/cplex/matheus_model_2/preprocessing.hpp"
#include "models/gvrp_models/cplex/matheus_model_2/invalid_edge_preprocessing.hpp"
#include "utils/util.hpp"

#include <iostream>
#include <list>

using namespace std;
using namespace utils;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex::matheus_model_2;

Invalid_edge_preprocessing::Invalid_edge_preprocessing (Matheus_model_2& matheus_model_2) : Preprocessing (matheus_model_2) {
  if (matheus_model_2.instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 3' only applies for metric instances");
}

void Invalid_edge_preprocessing::add () {
  //invalid edges
  list<pair<int, int>> edges = get_invalid_edges_1(matheus_model_2.instance);
  matheus_model_2.nPreprocessings1 = edges.size();
  for (auto const& [i, j] : edges) {
    auto c1 = matheus_model_2.customersC0Indexes.find(i),
         c2 = matheus_model_2.customersC0Indexes.find(j);
    auto f1 = matheus_model_2.afssF0Indexes.find(i),
         f2 = matheus_model_2.afssF0Indexes.find(j);
    //both are customers
    if (c1 != matheus_model_2.customersC0Indexes.end() && c2 != matheus_model_2.customersC0Indexes.end()) 
      matheus_model_2.model.add(matheus_model_2.x[c1->second][c2->second] == 0);
    //first is afs and second is customer
    if (f1 != matheus_model_2.afssF0Indexes.end() && c2 != matheus_model_2.customersC0Indexes.end()) 
      for (int i = 0; i < matheus_model_2.c0.size(); ++i)
        matheus_model_2.model.add(matheus_model_2.y[i][f1->second][c2->second] == 0);
    //first is customer and second is afs 
    if (c1 != matheus_model_2.customersC0Indexes.end() && f2 != matheus_model_2.afssF0Indexes.end()) 
      for (int i = 0; i < matheus_model_2.c0.size(); ++i)
        matheus_model_2.model.add(matheus_model_2.y[c1->second][f2->second][i] == 0);
    if (f1 != matheus_model_2.afssF0Indexes.end() && f2 != matheus_model_2.afssF0Indexes.end()) 
      for (int i_ = 0; i_ < matheus_model_2.c0.size(); ++i_)
        for (int j_ = 0; j_ < matheus_model_2.c0.size(); ++j_)
          for (int k = 0; k < matheus_model_2.c0.size(); ++k)
            matheus_model_2.model.add(matheus_model_2.y[i_][f1->second][j_] + matheus_model_2.y[j_][f2->second][k] <= 1);
  }
}
