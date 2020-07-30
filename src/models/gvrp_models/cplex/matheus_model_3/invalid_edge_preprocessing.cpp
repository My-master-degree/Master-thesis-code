#include "models/gvrp_models/cplex/matheus_model_3/matheus_model_3.hpp"
#include "models/gvrp_models/cplex/matheus_model_3/preprocessing.hpp"
#include "models/gvrp_models/cplex/matheus_model_3/invalid_edge_preprocessing.hpp"
#include "utils/util.hpp"

#include <list> 

using namespace utils;
using namespace std;
using namespace models::gvrp_models::cplex::matheus_model_3;

Invalid_edge_preprocessing::Invalid_edge_preprocessing (Matheus_model_3& matheus_model_3) : Preprocessing (matheus_model_3) {}

void Invalid_edge_preprocessing::add () {
  list<pair<int, int>> edges = get_invalid_edges_1(matheus_model_3.instance);
  matheus_model_3.nPreprocessings1 = edges.size();
  for (auto const& [i, j] : edges) {
    auto f1 = matheus_model_3.afs_dummies.find(i),
         f2 = matheus_model_3.afs_dummies.find(j);
    //both are customers
    if (f1 == matheus_model_3.afs_dummies.end() && f2 == matheus_model_3.afs_dummies.end()) 
      matheus_model_3.model.add(matheus_model_3.x[i][j] == 0);
    //first is afs and second is customer
    if (f1 != matheus_model_3.afs_dummies.end() && f2 == matheus_model_3.afs_dummies.end()) 
      for (int i_dummy : f1->second)
        matheus_model_3.model.add(matheus_model_3.x[i_dummy][j] == 0);
    //first is customer and second is afs 
    if (f1 == matheus_model_3.afs_dummies.end() && f2 != matheus_model_3.afs_dummies.end()) 
      for (int j_dummy : f2->second)
        matheus_model_3.model.add(matheus_model_3.x[i][j_dummy] == 0);
    if (f1 != matheus_model_3.afs_dummies.end() && f2 != matheus_model_3.afs_dummies.end()) 
      for (int i_dummy : f1->second)
        for (int j_dummy : f2->second)
        matheus_model_3.model.add(matheus_model_3.x[i_dummy][j_dummy] == 0);
  }
}
