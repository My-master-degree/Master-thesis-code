#include "models/gvrp_models/gvrp_feasible_solution_heuristic.hpp"
#include "models/gvrp_models/cplex/matheus_model_5/matheus_model_5.hpp"
#include "models/gvrp_models/cplex/matheus_model_5/preprocessing.hpp"
#include "models/gvrp_models/cplex/matheus_model_5/invalid_edge_preprocessing_5.hpp"
#include "utils/util.hpp"

#include <list>

using namespace std;
using namespace utils;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex::matheus_model_5;

Invalid_edge_preprocessing_5::Invalid_edge_preprocessing_5 (Matheus_model_5& matheus_model_5) : Preprocessing (matheus_model_5) {
  if (matheus_model_5.instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 5' only applies for metric instances");
}

void Invalid_edge_preprocessing_5::add () {
  for (int i = 0; i < matheus_model_5.c0.size(); ++i) {
    const Vertex * vertexI = matheus_model_5.c0[i];
    for (int j = 0; j < matheus_model_5.c0.size(); ++j) {
      const Vertex * vertexJ = matheus_model_5.c0[j];
      for (int f = 0; f < matheus_model_5._f.size(); ++f) {
        const Vertex * vertexF = matheus_model_5._f[f];
        for (int r = 0; r < matheus_model_5._f.size(); ++r) {
          const Vertex * vertexR = matheus_model_5._f[r];
          for (int f_ = 0; f_ < matheus_model_5._f.size(); ++f_) {
            const Vertex * vertexF_ = matheus_model_5._f[f_];
            for (int r_ = 0; r_ < matheus_model_5._f.size(); ++r_) {
              const Vertex * vertexR_ = matheus_model_5._f[r_];
              if ((f_ != f || r_ != r) && 
                  matheus_model_5.customerToAfsFuel(i, f_) <= matheus_model_5.customerToAfsFuel(i, f) &&
                  matheus_model_5.afsToCustomerFuel(r_, j) <= matheus_model_5.afsToCustomerFuel(r, j) &&
                  matheus_model_5.instance.time(vertexI->id, vertexF_->id) <= matheus_model_5.instance.time(vertexI->id, vertexF->id) &&
                  matheus_model_5.instance.time(vertexR_->id, vertexJ->id) <= matheus_model_5.instance.time(vertexR->id, vertexJ->id) &&
                  matheus_model_5.instance.distances[vertexI->id][vertexF_->id] <= matheus_model_5.instance.distances[vertexI->id][vertexF->id] &&
                  matheus_model_5.instance.distances[vertexR_->id][vertexJ->id] <= matheus_model_5.instance.distances[vertexR->id][vertexJ->id]) {
                //get f_, and r_ indexes
                int f1, r1, f2, r2;
                for (int k = 0; k < matheus_model_5.gvrp_afs_tree->f0.size(); ++k) {
                  if (matheus_model_5.gvrp_afs_tree->f0[k]->id == vertexF->id)
                    f1 = k;
                  if (matheus_model_5.gvrp_afs_tree->f0[k]->id == vertexR->id)
                    r1 = k;
                  if (matheus_model_5.gvrp_afs_tree->f0[k]->id == vertexF_->id)
                    f2 = k;
                  if (matheus_model_5.gvrp_afs_tree->f0[k]->id == vertexR_->id)
                    r2 = k;
                }
                //check time
                if (matheus_model_5.gvrp_afs_tree->pairTimes[f2][r2] <= matheus_model_5.gvrp_afs_tree->pairTimes[f1][r1] &&
                    matheus_model_5.gvrp_afs_tree->pairCosts[f2][r2] <= matheus_model_5.gvrp_afs_tree->pairCosts[f1][r1]) { 
                  ++matheus_model_5.nPreprocessings5;
                  matheus_model_5.model.add(matheus_model_5.y[i][f][r][j] == 0);
                  f_ = matheus_model_5.gvrp_afs_tree->f0.size();
                  break;
                }
              }
            }
          }
        }
      }
    }
  }
}
