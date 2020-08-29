#include "models/gvrp_models/cplex/matheus_model/matheus_model.hpp"
#include "models/gvrp_models/gvrp_feasible_solution_heuristic.hpp"
#include "models/gvrp_models/cplex/matheus_model/preprocessing.hpp"
#include "models/gvrp_models/cplex/matheus_model/invalid_edge_preprocessing.hpp"
#include "utils/util.hpp"

#include <iostream>
#include <list>

using namespace std;
using namespace utils;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex::matheus_model;

Invalid_edge_preprocessing::Invalid_edge_preprocessing (Matheus_model& matheus_model) : Preprocessing (matheus_model) {
  if (matheus_model.instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 3' only applies for metric instances");
}

void Invalid_edge_preprocessing::add () {
  //invalid edges
  list<pair<int, int>> edges = get_invalid_edges_1(matheus_model.instance);
  matheus_model.nPreprocessings1 = 0;
  for (auto const& [i, j] : edges) {
    auto c1 = matheus_model.customersC0Indexes.find(i),
         c2 = matheus_model.customersC0Indexes.find(j);
    auto f1 = matheus_model.afssF0Indexes.find(i),
         f2 = matheus_model.afssF0Indexes.find(j);
    //both are customers
    if (c1 != matheus_model.customersC0Indexes.end() && c2 != matheus_model.customersC0Indexes.end()) {
      matheus_model.model.add(matheus_model.x[c1->second][c2->second] == 0);
      ++matheus_model.nPreprocessings1;
    }
    //first is afs and second is customer
    if (f1 != matheus_model.afssF0Indexes.end() && c2 != matheus_model.customersC0Indexes.end()) 
      for (int i = 0; i < matheus_model.c0.size(); ++i) {
        matheus_model.model.add(matheus_model.y[i][f1->second][c2->second] == 0);
        ++matheus_model.nPreprocessings1;
      }
    //first is customer and second is afs 
    if (c1 != matheus_model.customersC0Indexes.end() && f2 != matheus_model.afssF0Indexes.end()) 
      for (int i = 0; i < matheus_model.c0.size(); ++i) {
        matheus_model.model.add(matheus_model.y[c1->second][f2->second][i] == 0);
        ++matheus_model.nPreprocessings1;
      }
  }
}
