#include "models/gvrp_models/gvrp_feasible_solution_heuristic.hpp"
#include "models/gvrp_models/cplex/emh_model/emh_model.hpp"
#include "models/gvrp_models/cplex/emh_model/preprocessing.hpp"
#include "models/gvrp_models/cplex/emh_model/invalid_edge_preprocessing_4.hpp"
#include "utils/util.hpp"

#include <list>

using namespace std;
using namespace utils;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex::emh_model;

Invalid_edge_preprocessing_4::Invalid_edge_preprocessing_4 (EMH_model& emh_model) : Preprocessing (emh_model) {
  if (emh_model.instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 4' only applies for metric instances");
}

void Invalid_edge_preprocessing_4::add () {
  list<pair<int, int>> edges = get_invalid_edges_4(emh_model.instance, *emh_model.gvrp_afs_tree);
  for (auto const& [i, j] : edges)
    //first is afs and second is customer
    if (emh_model.afs_dummies.count(i)) 
      for (int i_dummy : emh_model.afs_dummies[i])
        emh_model.model.add(emh_model.x[i_dummy][j] == 0);
  //first is customer and second is afs 
    else if (emh_model.afs_dummies.count(j)) 
      for (int j_dummy : emh_model.afs_dummies[j])
        emh_model.model.add(emh_model.x[i][j_dummy] == 0);
}
