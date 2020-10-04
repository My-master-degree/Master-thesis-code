#include "models/gvrp_models/gvrp_feasible_solution_heuristic.hpp"
#include "models/gvrp_models/cplex/matheus_model_3/matheus_model_3.hpp"
#include "models/gvrp_models/cplex/matheus_model_3/preprocessing.hpp"
#include "models/gvrp_models/cplex/matheus_model_3/invalid_edge_preprocessing_4.hpp"
#include "utils/util.hpp"

#include <list>

using namespace std;
using namespace utils;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex::matheus_model_3;

Invalid_edge_preprocessing_4::Invalid_edge_preprocessing_4 (Matheus_model_3& matheus_model_3) : Preprocessing (matheus_model_3) {
  if (matheus_model_3.instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 4' only applies for metric instances");
}

void Invalid_edge_preprocessing_4::add () {
  list<pair<int, int>> edges = get_invalid_edges_4(matheus_model_3.instance, *matheus_model_3.gvrp_afs_tree);
  matheus_model_3.nPreprocessings4 = 0;
  for (auto const& [i, j] : edges) {
    auto f1 = matheus_model_3.afs_dummies.find(i),
         f2 = matheus_model_3.afs_dummies.find(j);
    //first is afs and second is customer
    if (f1 != matheus_model_3.afs_dummies.end()) 
      for (int i_dummy : f1->second) {
        matheus_model_3.model.add(matheus_model_3.x[i_dummy][j] == 0);
        ++matheus_model_3.nPreprocessings4;
      }
    //first is customer and second is afs 
    else if (f2 != matheus_model_3.afs_dummies.end()) 
      for (int j_dummy : f2->second) {
        matheus_model_3.model.add(matheus_model_3.x[i][j_dummy] == 0);
        ++matheus_model_3.nPreprocessings4;
      }
  }
}
