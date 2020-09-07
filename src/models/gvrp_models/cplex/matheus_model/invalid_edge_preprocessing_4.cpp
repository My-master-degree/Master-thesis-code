#include "models/gvrp_models/gvrp_feasible_solution_heuristic.hpp"
#include "models/gvrp_models/cplex/matheus_model/matheus_model.hpp"
#include "models/gvrp_models/cplex/matheus_model/preprocessing.hpp"
#include "models/gvrp_models/cplex/matheus_model/invalid_edge_preprocessing_4.hpp"
#include "utils/util.hpp"

#include <list>

using namespace std;
using namespace utils;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex::matheus_model;

Invalid_edge_preprocessing_4::Invalid_edge_preprocessing_4 (Matheus_model& matheus_model) : Preprocessing (matheus_model) {
  if (matheus_model.instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 4' only applies for metric instances");
}

void Invalid_edge_preprocessing_4::add () {
  for (int i = 0; i < matheus_model.c0.size(); ++i) {
    const Vertex * vertexI = matheus_model.c0[i];
    for (int j = 0; j < matheus_model.c0.size(); ++j) {
      const Vertex * vertexJ = matheus_model.c0[j];
      for (int f_ = 0; f_ < matheus_model.f0.size(); ++f_) {
        bool valid = false;
        for (int f = 0; f < matheus_model.f0.size(); ++f) {
          const Vertex * vertexF = matheus_model.f0[f];
          for (int r = 0; r < matheus_model.f0.size(); ++r) {
            const Vertex * vertexR = matheus_model.f0[r];
            if (matheus_model.afsToCustomerFuel(f, i) + matheus_model.customerToAfsFuel(i, f_) <= matheus_model.instance.vehicleFuelCapacity 
                && matheus_model.afsToCustomerFuel(f_, j) + matheus_model.customerToAfsFuel(j, r) <= matheus_model.instance.vehicleFuelCapacity                      
                && matheus_model.gvrp_afs_tree->times[f] + matheus_model.instance.time(vertexF->id, vertexI->id) + matheus_model.time(i, f_, j) + vertexJ->serviceTime + matheus_model.instance.time(vertexJ->id, vertexR->id) + matheus_model.gvrp_afs_tree->times[r] <= matheus_model.instance.timeLimit) { 
              valid = true;
              f = matheus_model.f0.size();
              break;
            }
          }
        }
        if (!valid) {
          matheus_model.model.add(matheus_model.y[i][f_][j] == 0);
          ++matheus_model.nPreprocessings4;
        } else {
          const Vertex * vertexF_ = matheus_model.f0[f_];
          for (int f = 0; f < matheus_model.f0.size(); ++f) {
            const Vertex * vertexF = matheus_model.f0[f];
            if (f_ != f &&
                matheus_model.customerToAfsFuel(i, f) <= matheus_model.customerToAfsFuel(i, f_) && 
                matheus_model.afsToCustomerFuel(f, j) <= matheus_model.afsToCustomerFuel(f_, j) &&
                matheus_model.instance.time(vertexI->id, vertexF->id) <= matheus_model.instance.time(vertexI->id, vertexF_->id) && 
                matheus_model.instance.time(vertexF->id, vertexJ->id) <= matheus_model.instance.time(vertexF_->id, vertexJ->id) && 
                matheus_model.instance.distances[vertexI->id][vertexF->id] <= matheus_model.instance.distances[vertexI->id][vertexF_->id] && 
                matheus_model.instance.distances[vertexF->id][vertexJ->id] <= matheus_model.instance.distances[vertexF_->id][vertexJ->id]) {
              ++matheus_model.nPreprocessings4;
              matheus_model.model.add(matheus_model.y[i][f_][j] == 0);
              break;
            }
          }
        }
      }
    }
  }
}
