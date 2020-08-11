#include "models/gvrp_models/gvrp_feasible_solution_heuristic.hpp"
#include "models/gvrp_models/cplex/matheus_model_4/matheus_model_4.hpp"
#include "models/gvrp_models/cplex/matheus_model_4/preprocessing.hpp"
#include "models/gvrp_models/cplex/matheus_model_4/invalid_edge_preprocessing_4.hpp"
#include "utils/util.hpp"

#include <list>

using namespace std;
using namespace utils;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex::matheus_model_4;

Invalid_edge_preprocessing_4::Invalid_edge_preprocessing_4 (Matheus_model_4& matheus_model_4) : Preprocessing (matheus_model_4) {
  if (matheus_model_4.instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 4' only applies for metric instances");
}

void Invalid_edge_preprocessing_4::add () {
  for (int i = 0; i < matheus_model_4.c0.size(); ++i) {
    const Vertex * vertexI = matheus_model_4.c0[i];
    for (int j = 0; j < matheus_model_4.c0.size(); ++j) {
      const Vertex * vertexJ = matheus_model_4.c0[j];
      for (int f_ = 0; f_ < matheus_model_4._f.size(); ++f_) {
        for (int r_ = 0; r_ < matheus_model_4._f.size(); ++r_) {
          bool valid = false;
          for (int f = 0; f < matheus_model_4.gvrp_afs_tree->f0.size(); ++f) {
            const Vertex * vertexF = matheus_model_4.gvrp_afs_tree->f0[f];
            for (int r = 0; r < matheus_model_4.gvrp_afs_tree->f0.size(); ++r) {
              const Vertex * vertexR = matheus_model_4.gvrp_afs_tree->f0[r];
              if (matheus_model_4.instance.fuel(vertexF->id, i) + matheus_model_4.customerToAfsFuel(i, f_) <= matheus_model_4.instance.vehicleFuelCapacity 
                  && matheus_model_4.afsToCustomerFuel(r_, j) + matheus_model_4.instance.fuel(j, vertexR->id) <= matheus_model_4.instance.vehicleFuelCapacity) {
                //get f_, and r_ indexes
                int f__, r__;
                for (int k = 0; k < matheus_model_4.gvrp_afs_tree->f0.size(); ++k) {
                  if (matheus_model_4.gvrp_afs_tree->f0[k]->id == vertexF->id)
                    f__ = k;
                  if (matheus_model_4.gvrp_afs_tree->f0[k]->id == vertexR->id)
                    r__ = k;
                }
                //check time
                if (matheus_model_4.gvrp_afs_tree->times[f__] + matheus_model_4.instance.time(vertexF->id, vertexI->id) + matheus_model_4.time(i, f_, r_, j) + vertexJ->serviceTime + matheus_model_4.instance.time(vertexJ->id, vertexR->id) + matheus_model_4.gvrp_afs_tree->times[r__] <= matheus_model_4.instance.timeLimit) { 
                  valid = true;
                  f = matheus_model_4._f.size();
                  break;
                }
              }
            }
          }
          if (!valid) {
            matheus_model_4.model.add(matheus_model_4.y[i][f_][r_][j] == 0);
            ++matheus_model_4.nPreprocessings4;
          } 
        }
      }
    }
  }
}
