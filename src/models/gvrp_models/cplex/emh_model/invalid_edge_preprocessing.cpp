#include "models/gvrp_models/cplex/emh_model/emh_model.hpp"
#include "models/gvrp_models/cplex/emh_model/preprocessing.hpp"
#include "models/gvrp_models/cplex/emh_model/invalid_edge_preprocessing.hpp"
#include "utils/util.hpp"

#include <list> 

using namespace utils;
using namespace std;
using namespace models::gvrp_models::cplex::emh_model;

Invalid_edge_preprocessing::Invalid_edge_preprocessing (EMH_model& emh_model) : Preprocessing (emh_model) {}

void Invalid_edge_preprocessing::add () {
  list<pair<int, int>> edges = get_invalid_edges_1(emh_model.instance);
  for (auto const& [i, j] : edges) {
    //both are customers
    if (!emh_model.afs_dummies.count(i) && !emh_model.afs_dummies.count(j)) 
      emh_model.model.add(emh_model.x[i][j] == 0);
    //first is afs and second is customer
    else if (emh_model.afs_dummies.count(i) && !emh_model.afs_dummies.count(j)) 
      for (int i_dummy : emh_model.afs_dummies[i])
        emh_model.model.add(emh_model.x[i_dummy][j] == 0);
    //first is customer and second is afs 
    else if (!emh_model.afs_dummies.count(i) && emh_model.afs_dummies.count(j)) 
      for (int j_dummy : emh_model.afs_dummies[j])
        emh_model.model.add(emh_model.x[i][j_dummy] == 0);
    else 
      for (int i_dummy : emh_model.afs_dummies[i])
        for (int j_dummy : emh_model.afs_dummies[j])
        emh_model.model.add(emh_model.x[i_dummy][j_dummy] == 0);
  }
}
