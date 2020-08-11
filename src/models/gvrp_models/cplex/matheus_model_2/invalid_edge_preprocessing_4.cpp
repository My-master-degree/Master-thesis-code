#include "models/gvrp_models/gvrp_feasible_solution_heuristic.hpp"
#include "models/gvrp_models/cplex/matheus_model_2/matheus_model_2.hpp"
#include "models/gvrp_models/cplex/matheus_model_2/preprocessing.hpp"
#include "models/gvrp_models/cplex/matheus_model_2/invalid_edge_preprocessing_4.hpp"
#include "utils/util.hpp"

#include <list>
#include <float.h>

using namespace std;
using namespace utils;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex::matheus_model_2;

Invalid_edge_preprocessing_4::Invalid_edge_preprocessing_4 (Matheus_model_2& matheus_model_2) : Preprocessing (matheus_model_2) {
  if (matheus_model_2.instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 4' only applies for metric instances");
}

void Invalid_edge_preprocessing_4::add () {
  for (int i = 0; i < matheus_model_2.c0.size(); ++i) {
    const Vertex * vertexI = matheus_model_2.c0[i];
    for (int j = 0; j < matheus_model_2.c0.size(); ++j) {
      const Vertex * vertexJ = matheus_model_2.c0[j];
      for (int f_ = 0; f_ < matheus_model_2.f0.size(); ++f_) {
        bool valid = false;
        for (int f = 0; f < matheus_model_2.f0.size(); ++f) {
          const Vertex * vertexF = matheus_model_2.f0[f];
          for (int r = 0; r < matheus_model_2.f0.size(); ++r) {
            const Vertex * vertexR = matheus_model_2.f0[r];
            if (matheus_model_2.afsToCustomerFuel(f, i) + matheus_model_2.customerToAfsFuel(i, f_) <= matheus_model_2.instance.vehicleFuelCapacity 
                && matheus_model_2.afsToCustomerFuel(f_, j) + matheus_model_2.customerToAfsFuel(j, r) <= matheus_model_2.instance.vehicleFuelCapacity                      && matheus_model_2.gvrp_afs_tree->times[f] + matheus_model_2.instance.time(vertexF->id, vertexI->id) + matheus_model_2.time(i, f_, j) + vertexJ->serviceTime + matheus_model_2.instance.time(vertexJ->id, vertexR->id) + matheus_model_2.gvrp_afs_tree->times[r] <= matheus_model_2.instance.timeLimit) { 
              valid = true;
              f = matheus_model_2.f0.size();
              break;
            }
          }
        }
        if (!valid) {
          matheus_model_2.model.add(matheus_model_2.y[i][f_][j] == 0);
          ++matheus_model_2.nPreprocessings4;
        }
      }
    }
  }
}
