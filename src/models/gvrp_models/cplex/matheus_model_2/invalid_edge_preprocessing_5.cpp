#include "models/gvrp_models/gvrp_feasible_solution_heuristic.hpp"
#include "models/gvrp_models/cplex/matheus_model_2/matheus_model_2.hpp"
#include "models/gvrp_models/cplex/matheus_model_2/preprocessing.hpp"
#include "models/gvrp_models/cplex/matheus_model_2/invalid_edge_preprocessing_5.hpp"
#include "utils/util.hpp"

#include <list>

using namespace std;
using namespace utils;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex::matheus_model_2;

Invalid_edge_preprocessing_5::Invalid_edge_preprocessing_5 (Matheus_model_2& matheus_model_2) : Preprocessing (matheus_model_2) {
  if (matheus_model_2.instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 5' only applies for metric instances");
}

void Invalid_edge_preprocessing_5::add () {
  for (int i = 0; i < matheus_model_2.c0.size(); ++i) {
    const Vertex * vertexI = matheus_model_2.c0[i];
    for (int j = 0; j < matheus_model_2.c0.size(); ++j) {
      const Vertex * vertexJ = matheus_model_2.c0[j];
      for (int f = 0; f < matheus_model_2.f0.size(); ++f) {
        const Vertex * vertexF = matheus_model_2.f0[f];
        for (int f_ = 0; f_ < matheus_model_2.f0.size(); ++f_) {
          const Vertex * vertexF_ = matheus_model_2.f0[f_];
          if (f != f_ &&
              matheus_model_2.customerToAfsFuel(i, f_) <= matheus_model_2.customerToAfsFuel(i, f) && 
              matheus_model_2.afsToCustomerFuel(f_, j) <= matheus_model_2.afsToCustomerFuel(f, j) &&
              matheus_model_2.instance.time(vertexI->id, vertexF_->id) <= matheus_model_2.instance.time(vertexI->id, vertexF->id) && 
              matheus_model_2.instance.time(vertexF_->id, vertexJ->id) <= matheus_model_2.instance.time(vertexF->id, vertexJ->id) && 
              matheus_model_2.instance.distances[vertexI->id][vertexF_->id] <= matheus_model_2.instance.distances[vertexI->id][vertexF->id] && 
              matheus_model_2.instance.distances[vertexF_->id][vertexJ->id] <= matheus_model_2.instance.distances[vertexF->id][vertexJ->id]) {
            ++matheus_model_2.nPreprocessings5;
            matheus_model_2.model.add(matheus_model_2.y[i][f][j] == 0);
            break;
          }
        }
      }
    }
  }
}
